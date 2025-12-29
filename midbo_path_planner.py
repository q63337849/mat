"""
Python translation of the improved MIDBO algorithm for 3D trajectory planning.

This module mirrors the MATLAB implementation in ``AlgorithmCode/MIDBO.m`` and
keeps the path-planning cost function compatible with ``Cost_SPSO_rect.m``.
The planner samples spline-interpolated paths, evaluates obstacle avoidance,
height constraints, and smoothness, and searches for an optimal waypoint set
using MIDBO.
"""

from __future__ import annotations

import importlib.util
import math
from dataclasses import dataclass
from typing import Callable, Iterable, Tuple

import numpy as np

# Optional SciPy support for cubic spline interpolation.
_SCIPY_AVAILABLE = importlib.util.find_spec("scipy.interpolate") is not None
if _SCIPY_AVAILABLE:
    from scipy.interpolate import CubicSpline
else:  # pragma: no cover - fallback when SciPy is unavailable
    CubicSpline = None


@dataclass
class TrajectoryEnvironment:
    """Environment configuration for trajectory planning.

    Attributes
    ----------
    start_pos : Tuple[float, float, float]
        Starting position of the vehicle.
    goal_pos : Tuple[float, float, float]
        Goal position of the vehicle.
    map_range : Tuple[float, float, float]
        Map extents in X, Y, Z (positive values).
    obstacles : np.ndarray
        Axis-aligned bounding boxes shaped ``(n, 6)`` with columns
        ``[x, y, z, width, length, height]``.
    waypoint_count : int
        Number of intermediate waypoints to optimize.
    weights : Tuple[float, float, float, float]
        Weights ``(b1, b2, b3, b4)`` for length, threat, altitude, and
        smoothness costs respectively.
    threat_buffer : float
        Distance added around obstacles for collision penalty.
    danger_zone : float
        Width of the linear penalty zone around obstacles.
    min_height : float
        Minimum allowed altitude.
    max_height : float
        Maximum allowed altitude.
    turn_weight : float
        Weight for horizontal turning angles in the smoothness term.
    climb_weight : float
        Weight for climb angle changes in the smoothness term.
    sample_count : int
        Number of samples used to evaluate the spline path.
    """

    start_pos: Tuple[float, float, float]
    goal_pos: Tuple[float, float, float]
    map_range: Tuple[float, float, float]
    obstacles: np.ndarray
    waypoint_count: int
    weights: Tuple[float, float, float, float] = (5.0, 1.0, 10.0, 1.0)
    threat_buffer: float = 1.0
    danger_zone: float = 8.0
    min_height: float = 5.0
    max_height: float = 100.0
    turn_weight: float = 1.0
    climb_weight: float = 1.0
    sample_count: int = 120

    @property
    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Lower and upper bounds for optimization variables."""

        lb = np.zeros(3 * self.waypoint_count)
        ub = np.concatenate(
            [
                np.full(self.waypoint_count, self.map_range[0]),
                np.full(self.waypoint_count, self.map_range[1]),
                np.full(self.waypoint_count, self.map_range[2]),
            ]
        )
        return lb, ub

    def cost(self, position_vector: np.ndarray) -> float:
        """Evaluate the trajectory cost for a flattened waypoint vector."""

        n = self.waypoint_count
        x_seq = np.concatenate(([self.start_pos[0]], position_vector[:n], [self.goal_pos[0]]))
        y_seq = np.concatenate(
            ([self.start_pos[1]], position_vector[n : 2 * n], [self.goal_pos[1]])
        )
        z_seq = np.concatenate(
            ([self.start_pos[2]], position_vector[2 * n : 3 * n], [self.goal_pos[2]])
        )

        indices = np.linspace(0.0, 1.0, num=x_seq.size)
        samples = np.linspace(0.0, 1.0, num=self.sample_count)

        if CubicSpline is not None:
            x_path = CubicSpline(indices, x_seq)(samples)
            y_path = CubicSpline(indices, y_seq)(samples)
            z_path = CubicSpline(indices, z_seq)(samples)
        else:
            x_path = np.interp(samples, indices, x_seq)
            y_path = np.interp(samples, indices, y_seq)
            z_path = np.interp(samples, indices, z_seq)

        path = np.stack([x_path, y_path, z_path], axis=1)

        if np.any(path[:, 0] < 0) or np.any(path[:, 0] > self.map_range[0]):
            return math.inf
        if np.any(path[:, 1] < 0) or np.any(path[:, 1] > self.map_range[1]):
            return math.inf
        if np.any(path[:, 2] < 0) or np.any(path[:, 2] > self.map_range[2]):
            return math.inf

        length_cost = self._path_length(path)
        threat_cost = self._threat_cost(path)
        altitude_cost = self._altitude_cost(path[:, 2])
        if math.isinf(altitude_cost):
            return math.inf
        smoothness_cost = self._smoothness_cost(path)

        b1, b2, b3, b4 = self.weights
        return b1 * length_cost + b2 * threat_cost + b3 * altitude_cost + b4 * smoothness_cost

    def _path_length(self, path: np.ndarray) -> float:
        segments = np.diff(path, axis=0)
        return float(np.linalg.norm(segments, axis=1).sum())

    def _threat_cost(self, path: np.ndarray) -> float:
        cost = 0.0
        for j in range(path.shape[0] - 1):
            p0 = path[j]
            p1 = path[j + 1]
            for box in self.obstacles:
                expanded = _inflate_aabb(box, self.threat_buffer)
                if _segment_aabb_intersect(p0, p1, expanded):
                    return math.inf
                distance = _segment_aabb_distance(p0, p1, expanded)
                if distance < self.danger_zone:
                    cost += self.danger_zone - distance
        return cost

    def _altitude_cost(self, altitudes: np.ndarray) -> float:
        midpoint = 0.5 * (self.max_height + self.min_height)
        cost = 0.0
        for altitude in altitudes:
            if altitude < self.min_height or altitude > self.max_height:
                return math.inf
            cost += abs(altitude - midpoint)
        return cost

    def _smoothness_cost(self, path: np.ndarray) -> float:
        phi_values = []
        psi_values = []

        for j in range(path.shape[0] - 1):
            segment = path[j + 1] - path[j]
            horizontal = np.array([segment[0], segment[1], 0.0])
            denominator = np.linalg.norm(horizontal)
            if denominator == 0.0:
                psi_values.append(math.copysign(math.pi / 2.0, segment[2]))
            else:
                psi_values.append(math.atan(segment[2] / denominator))

        for j in range(path.shape[0] - 2):
            v1 = path[j + 1] - path[j]
            v2 = path[j + 2] - path[j + 1]
            v1_xy = np.array([v1[0], v1[1], 0.0])
            v2_xy = np.array([v2[0], v2[1], 0.0])
            norm_product = np.linalg.norm(v1_xy) * np.linalg.norm(v2_xy)
            if norm_product == 0.0:
                phi_values.append(0.0)
            else:
                cross_z = v1_xy[0] * v2_xy[1] - v1_xy[1] * v2_xy[0]
                dot_product = v1_xy[0] * v2_xy[0] + v1_xy[1] * v2_xy[1]
                phi_values.append(math.atan2(abs(cross_z), dot_product))

        phi_sum = sum(phi_values)
        psi_differences = np.diff(psi_values) if len(psi_values) > 1 else []
        psi_sum = float(np.abs(psi_differences).sum())

        return self.turn_weight * phi_sum + self.climb_weight * psi_sum


def midbo(
    population: int,
    iterations: int,
    lower_bound: Iterable[float],
    upper_bound: Iterable[float],
    dimension: int,
    objective: Callable[[np.ndarray], float],
    random_state: int | None = None,
) -> Tuple[float, np.ndarray, np.ndarray]:
    """Run the MIDBO algorithm.

    Parameters mirror the MATLAB implementation and return the best fitness,
    best position vector, and the convergence curve over iterations.
    """

    rng = np.random.default_rng(random_state)
    lower = np.asarray(lower_bound, dtype=float)
    upper = np.asarray(upper_bound, dtype=float)

    p_percent = 0.2
    producer_count = int(round(population * p_percent))

    population_matrix = lower + (upper - lower) * rng.random((population, dimension))
    fitness = np.array([objective(individual) for individual in population_matrix])

    personal_best_fit = fitness.copy()
    personal_best_pos = population_matrix.copy()
    best_index = int(np.argmin(fitness))
    global_best_fit = float(fitness[best_index])
    global_best_pos = population_matrix[best_index].copy()

    convergence = np.full(iterations, np.inf, dtype=float)
    convergence[0] = global_best_fit

    for t in range(iterations):
        current_iter = t + 1

        sorted_indices = np.argsort(personal_best_fit)
        elite_positions = personal_best_pos[sorted_indices[:3]]
        elite_mean = elite_positions.mean(axis=0)
        worst_index = int(np.argmax(personal_best_fit))
        worst_pos = personal_best_pos[worst_index]

        weight = max(0.7 - 0.5 * (current_iter / iterations), 0.3)
        guidance = weight * global_best_pos + (1.0 - weight) * elite_mean

        r2 = rng.random()
        for i in range(producer_count):
            if r2 < 0.9:
                direction = 2 * (rng.random() > 0.1) - 1
                candidate = (
                    personal_best_pos[i]
                    + 0.3 * np.abs(personal_best_pos[i] - worst_pos)
                    + direction * 0.1 * personal_best_pos[i]
                )
            else:
                candidate = personal_best_pos[i].copy()
            population_matrix[i] = _bounds(candidate, lower, upper)
            fitness[i] = objective(population_matrix[i])

        R = 1.0 - current_iter / iterations
        xnew1 = _bounds(guidance * (1.0 - R), lower, upper)
        xnew2 = _bounds(guidance * (1.0 + R), lower, upper)
        for i in range(producer_count, population):
            weight_i = i / max(population - 1, 1)
            candidate = weight_i * guidance + (
                rng.random(dimension) * (personal_best_pos[i] - xnew1)
                + rng.random(dimension) * (personal_best_pos[i] - xnew2)
            )
            population_matrix[i] = _bounds(candidate, xnew1, xnew2)
            fitness[i] = objective(population_matrix[i])

        for i in range(population):
            if fitness[i] < personal_best_fit[i]:
                personal_best_fit[i] = fitness[i]
                personal_best_pos[i] = population_matrix[i].copy()
            if personal_best_fit[i] < global_best_fit:
                global_best_fit = personal_best_fit[i]
                global_best_pos = personal_best_pos[i].copy()

        if current_iter <= round(0.2 * iterations):
            levy_prob = 0.7
            levy_scale = 0.25
        else:
            levy_prob = 0.3 + 0.4 * (current_iter / iterations)
            levy_scale = 0.14 + 0.12 * (current_iter / iterations)
        beta = 1.5
        sigma_levy = (
            math.gamma(1 + beta)
            * math.sin(math.pi * beta / 2)
            / (math.gamma((1 + beta) / 2) * beta * 2 ** ((beta - 1) / 2))
        ) ** (1 / beta)

        for i in range(population):
            if rng.random() < levy_prob:
                u = rng.standard_normal(dimension) * sigma_levy
                v = rng.standard_normal(dimension)
                step = u / np.abs(v) ** (1 / beta)
                levy_candidate = personal_best_pos[i] + levy_scale * step * (upper - lower)
                levy_candidate = _bounds(levy_candidate, lower, upper)
                levy_fit = objective(levy_candidate)
                if levy_fit < personal_best_fit[i]:
                    personal_best_fit[i] = levy_fit
                    personal_best_pos[i] = levy_candidate
                if levy_fit < global_best_fit:
                    global_best_fit = levy_fit
                    global_best_pos = levy_candidate

        for i in range(population):
            if rng.random() < 0.1:
                mutation = personal_best_pos[i] + 0.01 * rng.standard_normal(dimension) * personal_best_pos[i]
                mutation = _bounds(mutation, lower, upper)
                mut_fit = objective(mutation)
                if mut_fit < personal_best_fit[i]:
                    personal_best_fit[i] = mut_fit
                    personal_best_pos[i] = mutation
                if mut_fit < global_best_fit:
                    global_best_fit = mut_fit
                    global_best_pos = mutation

        stagnation_window = 3
        if current_iter > stagnation_window:
            recent = np.abs(convergence[max(0, t - stagnation_window) : t] - global_best_fit)
            if recent.size and np.all(recent < 1e-8):
                for i in range(population):
                    if rng.random() < 0.7:
                        mutation = personal_best_pos[i] + rng.standard_normal(dimension) * personal_best_pos[i]
                    else:
                        mutation = lower + (upper - lower) * rng.random(dimension)
                    mutation = _bounds(mutation, lower, upper)
                    mut_fit = objective(mutation)
                    if mut_fit < personal_best_fit[i]:
                        personal_best_fit[i] = mut_fit
                        personal_best_pos[i] = mutation
                    if mut_fit < global_best_fit:
                        global_best_fit = mut_fit
                        global_best_pos = mutation

        danger_k = 5
        danger_var_th = 1e-6
        danger_ratio = 0.2
        if current_iter > danger_k and (
            global_best_fit == convergence[max(0, t - danger_k)] or np.var(personal_best_fit) < danger_var_th
        ):
            danger_count = max(1, int(round(danger_ratio * population)))
            worst_indices = np.argpartition(personal_best_fit, -danger_count)[-danger_count:]
            for idx in worst_indices:
                personal_best_pos[idx] = lower + (upper - lower) * rng.random(dimension)
                personal_best_fit[idx] = objective(personal_best_pos[idx])
                if personal_best_fit[idx] < global_best_fit:
                    global_best_fit = personal_best_fit[idx]
                    global_best_pos = personal_best_pos[idx]

        convergence[t] = global_best_fit

    return global_best_fit, global_best_pos, convergence


def plan_path_with_midbo(
    env: TrajectoryEnvironment,
    population: int = 30,
    iterations: int = 200,
    random_state: int | None = None,
) -> Tuple[float, np.ndarray, np.ndarray]:
    """Plan a trajectory using MIDBO for the provided environment."""

    lb, ub = env.bounds
    best_fit, best_pos, convergence = midbo(
        population=population,
        iterations=iterations,
        lower_bound=lb,
        upper_bound=ub,
        dimension=lb.size,
        objective=env.cost,
        random_state=random_state,
    )
    return best_fit, best_pos, convergence


def _bounds(vector: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    low = np.minimum(lower, upper)
    high = np.maximum(lower, upper)
    return np.clip(vector, low, high)


def _inflate_aabb(box: np.ndarray, radius: float) -> np.ndarray:
    expanded = box.copy()
    expanded[:3] -= radius
    expanded[3:] += 2 * radius
    return expanded


def _segment_aabb_intersect(p0: np.ndarray, p1: np.ndarray, box: np.ndarray) -> bool:
    b_min = box[:3]
    b_max = box[:3] + box[3:]
    direction = p1 - p0
    t0, t1 = 0.0, 1.0
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if p0[axis] < b_min[axis] or p0[axis] > b_max[axis]:
                return False
        else:
            inv_d = 1.0 / direction[axis]
            t_near = (b_min[axis] - p0[axis]) * inv_d
            t_far = (b_max[axis] - p0[axis]) * inv_d
            if t_near > t_far:
                t_near, t_far = t_far, t_near
            t0 = max(t0, t_near)
            t1 = min(t1, t_far)
            if t0 > t1:
                return False
    return True


def _segment_aabb_distance(p0: np.ndarray, p1: np.ndarray, box: np.ndarray) -> float:
    b_min = box[:3]
    b_max = box[:3] + box[3:]
    d_min = math.inf
    for t in np.linspace(0.0, 1.0, num=20):
        point = p0 + t * (p1 - p0)
        clamped = np.minimum(np.maximum(point, b_min), b_max)
        d_min = min(d_min, float(np.linalg.norm(point - clamped)))
    return d_min


__all__ = [
    "TrajectoryEnvironment",
    "midbo",
    "plan_path_with_midbo",
]
