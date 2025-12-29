"""Python rewrite of the MIDBO trajectory planning algorithm.

This module mirrors the MATLAB implementation in ``AlgorithmCode/MIDBO.m``
and pairs it with a pure-Python port of the path cost function used by the
original project (``Cost_SPSO_rect.m``).  No third-party packages are
required so the algorithm can run in restricted environments.

Example
-------
>>> from midbo_path_planning import (
...     MidboParameters, TrajectoryEnvironment, run_midbo_planner,
...     PRESET_OBSTACLES_F2,
... )
>>> env = TrajectoryEnvironment(
...     start_pos=(10, 10, 10),
...     goal_pos=(175, 175, 50),
...     map_range=(200, 200, 200),
...     boxes=PRESET_OBSTACLES_F2,
...     n_waypoints=2,
... )
>>> params = MidboParameters(population=30, iterations=200)
>>> best_cost, best_path, convergence = run_midbo_planner(env, params)

CLI usage
---------
``python -m midbo_path_planning --iterations 10 --population 10``
will run a short optimization with the fixed obstacle field from the MATLAB
code and print the resulting best path along with the convergence history.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from statistics import pvariance
from typing import Callable, Iterable, List, Sequence, Tuple

Number = float
Vector = List[Number]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _vector_add(a: Vector, b: Vector) -> Vector:
    return [x + y for x, y in zip(a, b)]


def _vector_sub(a: Vector, b: Vector) -> Vector:
    return [x - y for x, y in zip(a, b)]


def _vector_mul(a: Vector, scalar: Number) -> Vector:
    return [scalar * x for x in a]


def _vector_abs(a: Vector) -> Vector:
    return [abs(x) for x in a]


def _vector_mean(vectors: Sequence[Vector]) -> Vector:
    if not vectors:
        raise ValueError("Cannot average an empty sequence")
    n = len(vectors)
    dim = len(vectors[0])
    return [sum(vec[i] for vec in vectors) / n for i in range(dim)]


def _vector_bounds(x: Vector, lower: Vector, upper: Vector) -> Vector:
    return [min(max(val, l), u) for val, l, u in zip(x, lower, upper)]


def _vector_bounds_between(x: Vector, a: Vector, b: Vector) -> Vector:
    return [
        min(max(val, min(l, u)), max(l, u))
        for val, l, u in zip(x, a, b)
    ]


def _random_vector(lower: Vector, upper: Vector) -> Vector:
    return [l + (u - l) * random.random() for l, u in zip(lower, upper)]


# ---------------------------------------------------------------------------
# Natural cubic spline (1D) with "natural" boundary conditions.
# ---------------------------------------------------------------------------

class NaturalCubicSpline:
    """Lightweight natural cubic spline for 1D data.

    The implementation avoids external dependencies while mimicking the
    behaviour of MATLAB's ``spline`` with natural end conditions.
    """

    def __init__(self, x: Sequence[Number], y: Sequence[Number]):
        if len(x) != len(y):
            raise ValueError("x and y must have the same length")
        if len(x) < 2:
            raise ValueError("At least two points are required for a spline")
        if any(x[i] >= x[i + 1] for i in range(len(x) - 1)):
            raise ValueError("x must be strictly increasing")
        self.x = list(map(float, x))
        self.a = list(map(float, y))
        self._b, self._c, self._d = self._compute_coefficients()

    def _compute_coefficients(self) -> Tuple[List[float], List[float], List[float]]:
        n = len(self.x)
        h = [self.x[i + 1] - self.x[i] for i in range(n - 1)]

        alpha = [0.0] * n
        for i in range(1, n - 1):
            alpha[i] = (
                (3 / h[i]) * (self.a[i + 1] - self.a[i])
                - (3 / h[i - 1]) * (self.a[i] - self.a[i - 1])
            )

        l = [1.0] + [0.0] * (n - 1)
        mu = [0.0] * n
        z = [0.0] * n
        for i in range(1, n - 1):
            l[i] = 2 * (self.x[i + 1] - self.x[i - 1]) - h[i - 1] * mu[i - 1]
            mu[i] = h[i] / l[i]
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i]
        l[n - 1] = 1.0
        z[n - 1] = 0.0

        c = [0.0] * n
        b = [0.0] * (n - 1)
        d = [0.0] * (n - 1)
        for j in range(n - 2, -1, -1):
            c[j] = z[j] - mu[j] * c[j + 1]
            b[j] = (
                (self.a[j + 1] - self.a[j]) / h[j]
                - h[j] * (c[j + 1] + 2 * c[j]) / 3
            )
            d[j] = (c[j + 1] - c[j]) / (3 * h[j])
        return b, c[:-1], d

    def evaluate(self, x_new: Iterable[Number]) -> List[Number]:
        results: List[Number] = []
        for xn in x_new:
            idx = self._find_interval(xn)
            dx = xn - self.x[idx]
            results.append(
                self.a[idx]
                + self._b[idx] * dx
                + self._c[idx] * dx * dx
                + self._d[idx] * dx * dx * dx
            )
        return results

    def _find_interval(self, x_val: Number) -> int:
        # Clamp to valid range then find the right interval.
        if x_val <= self.x[0]:
            return 0
        if x_val >= self.x[-1]:
            return len(self.x) - 2
        low, high = 0, len(self.x) - 1
        while low <= high:
            mid = (low + high) // 2
            if self.x[mid] <= x_val < self.x[mid + 1]:
                return mid
            if x_val < self.x[mid]:
                high = mid - 1
            else:
                low = mid + 1
        return len(self.x) - 2


# ---------------------------------------------------------------------------
# Trajectory environment and cost evaluation
# ---------------------------------------------------------------------------

@dataclass
class TrajectoryEnvironment:
    start_pos: Tuple[Number, Number, Number]
    goal_pos: Tuple[Number, Number, Number]
    map_range: Tuple[Number, Number, Number]
    boxes: List[Tuple[Number, Number, Number, Number, Number, Number]]
    n_waypoints: int
    weight_length: Number = 5.0
    weight_threat: Number = 1.0
    weight_altitude: Number = 10.0
    weight_smooth: Number = 1.0
    uav_diameter: Number = 1.0
    threat_buffer: Number = 8.0
    min_altitude: Number = 5.0
    max_altitude: Number = 100.0
    smooth_turn_weight: Number = 1.0
    smooth_climb_weight: Number = 1.0
    nsample: int = 120

    def cost(self, candidate: Sequence[Number]) -> Number:
        path_points = self._sample_path(candidate)
        if self._out_of_bounds(path_points):
            return math.inf

        f1 = self._path_length(path_points)
        f2 = self._threat_penalty(path_points)
        if math.isinf(f2):
            return math.inf

        f3 = self._altitude_penalty(path_points)
        if math.isinf(f3):
            return math.inf

        f4 = self._smoothness_penalty(path_points)

        return (
            self.weight_length * f1
            + self.weight_threat * f2
            + self.weight_altitude * f3
            + self.weight_smooth * f4
        )

    # ----- path construction -------------------------------------------------
    def _sample_path(self, candidate: Sequence[Number]) -> List[Tuple[Number, Number, Number]]:
        n = self.n_waypoints
        if len(candidate) != 3 * n:
            raise ValueError(f"Candidate length must be {3*n}")

        x_seq = [self.start_pos[0]] + list(candidate[0:n]) + [self.goal_pos[0]]
        y_seq = [self.start_pos[1]] + list(candidate[n : 2 * n]) + [self.goal_pos[1]]
        z_seq = [self.start_pos[2]] + list(candidate[2 * n : 3 * n]) + [self.goal_pos[2]]

        k = len(x_seq)
        i_seq = [i / (k - 1) for i in range(k)]
        sample_points = [i / (self.nsample - 1) for i in range(self.nsample)]

        xspline = NaturalCubicSpline(i_seq, x_seq)
        yspline = NaturalCubicSpline(i_seq, y_seq)
        zspline = NaturalCubicSpline(i_seq, z_seq)

        x_vals = xspline.evaluate(sample_points)
        y_vals = yspline.evaluate(sample_points)
        z_vals = zspline.evaluate(sample_points)
        return list(zip(x_vals, y_vals, z_vals))

    def _out_of_bounds(self, points: Sequence[Tuple[Number, Number, Number]]) -> bool:
        for x, y, z in points:
            if not (0 <= x <= self.map_range[0]):
                return True
            if not (0 <= y <= self.map_range[1]):
                return True
            if not (0 <= z <= self.map_range[2]):
                return True
        return False

    # ----- cost components ---------------------------------------------------
    @staticmethod
    def _path_length(points: Sequence[Tuple[Number, Number, Number]]) -> Number:
        length = 0.0
        for p0, p1 in zip(points, points[1:]):
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            dz = p1[2] - p0[2]
            length += math.sqrt(dx * dx + dy * dy + dz * dz)
        return length

    def _threat_penalty(self, points: Sequence[Tuple[Number, Number, Number]]) -> Number:
        penalty = 0.0
        for p0, p1 in zip(points, points[1:]):
            for box in self.boxes:
                inflated = self._inflate_box(box, self.uav_diameter)
                if self._segment_intersects_box(p0, p1, inflated):
                    return math.inf
                distance = self._segment_box_distance(p0, p1, inflated)
                if distance < self.threat_buffer:
                    penalty += self.threat_buffer - distance
        return penalty

    def _altitude_penalty(self, points: Sequence[Tuple[Number, Number, Number]]) -> Number:
        mid_height = (self.max_altitude + self.min_altitude) / 2
        penalty = 0.0
        for _x, _y, z in points:
            if z < self.min_altitude or z > self.max_altitude:
                return math.inf
            penalty += abs(z - mid_height)
        return penalty

    def _smoothness_penalty(self, points: Sequence[Tuple[Number, Number, Number]]) -> Number:
        # Climb angle per segment
        climb_angles: List[Number] = []
        for p0, p1 in zip(points, points[1:]):
            vx, vy, vz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
            horiz_norm = math.sqrt(vx * vx + vy * vy)
            if horiz_norm == 0:
                climb_angles.append(math.pi / 2 if vz >= 0 else -math.pi / 2)
            else:
                climb_angles.append(math.atan(vz / horiz_norm))

        # Turn angle between successive horizontal projections
        turn_angles: List[Number] = []
        for (p0, p1), (p1b, p2) in zip(zip(points, points[1:]), zip(points[1:], points[2:])):
            v1x, v1y = p1[0] - p0[0], p1[1] - p0[1]
            v2x, v2y = p2[0] - p1b[0], p2[1] - p1b[1]
            norm1 = math.sqrt(v1x * v1x + v1y * v1y)
            norm2 = math.sqrt(v2x * v2x + v2y * v2y)
            if norm1 == 0 or norm2 == 0:
                turn_angles.append(0.0)
            else:
                cross_z = v1x * v2y - v1y * v2x
                dot = v1x * v2x + v1y * v2y
                turn_angles.append(math.atan2(abs(cross_z), dot))

        return (
            self.smooth_turn_weight * sum(turn_angles)
            + self.smooth_climb_weight * sum(abs(a - b) for a, b in zip(climb_angles, climb_angles[1:]))
        )

    # ----- geometry ---------------------------------------------------------
    @staticmethod
    def _inflate_box(box: Tuple[Number, Number, Number, Number, Number, Number], r: Number):
        x, y, z, w, l, h = box
        return (x - r, y - r, z - r, w + 2 * r, l + 2 * r, h + 2 * r)

    @staticmethod
    def _segment_intersects_box(p0, p1, box) -> bool:
        x, y, z, w, l, h = box
        bmin = (x, y, z)
        bmax = (x + w, y + l, z + h)
        d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
        t0, t1 = 0.0, 1.0
        for i in range(3):
            if abs(d[i]) < 1e-12:
                if p0[i] < bmin[i] or p0[i] > bmax[i]:
                    return False
            else:
                inv = 1.0 / d[i]
                t_near = (bmin[i] - p0[i]) * inv
                t_far = (bmax[i] - p0[i]) * inv
                if t_near > t_far:
                    t_near, t_far = t_far, t_near
                t0 = max(t0, t_near)
                t1 = min(t1, t_far)
                if t0 > t1:
                    return False
        return True

    @staticmethod
    def _segment_box_distance(p0, p1, box) -> Number:
        x, y, z, w, l, h = box
        bmin = (x, y, z)
        bmax = (x + w, y + l, z + h)
        samples = 20
        min_dist = math.inf
        for i in range(samples + 1):
            t = i / samples
            px = p0[0] + t * (p1[0] - p0[0])
            py = p0[1] + t * (p1[1] - p0[1])
            pz = p0[2] + t * (p1[2] - p0[2])
            qx = min(max(px, bmin[0]), bmax[0])
            qy = min(max(py, bmin[1]), bmax[1])
            qz = min(max(pz, bmin[2]), bmax[2])
            dist = math.sqrt((px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2)
            if dist < min_dist:
                min_dist = dist
        return min_dist


# ---------------------------------------------------------------------------
# MIDBO optimizer (Python port of AlgorithmCode/MIDBO.m)
# ---------------------------------------------------------------------------

@dataclass
class MidboParameters:
    population: int
    iterations: int
    lower_bounds: Sequence[Number] | None = None
    upper_bounds: Sequence[Number] | None = None
    random_seed: int | None = None


def midbo(
    population: int,
    iterations: int,
    lower_bounds: Sequence[Number],
    upper_bounds: Sequence[Number],
    dim: int,
    objective: Callable[[Vector], Number],
    random_seed: int | None = None,
) -> Tuple[Number, Vector, List[Number]]:
    """Run the MIDBO optimizer.

    Parameters match the MATLAB signature: ``pop, M, c, d, dim, fobj``.
    """

    if random_seed is not None:
        random.seed(random_seed)

    p_percent = 0.2
    p_num = max(1, round(population * p_percent))
    lb = list(lower_bounds)
    ub = list(upper_bounds)

    # Initialization
    x = [_random_vector(lb, ub) for _ in range(population)]
    fit = [objective(ind) for ind in x]
    p_fit = fit.copy()
    p_x = [ind.copy() for ind in x]
    f_min = min(fit)
    best_idx = fit.index(f_min)
    best_x = x[best_idx].copy()

    convergence = [math.inf for _ in range(iterations)]
    convergence[0] = f_min

    for t in range(iterations):
        # Elite guidance
        n_elite = min(3, population)
        elite_pairs = sorted(zip(p_fit, p_x), key=lambda pair: pair[0])[:n_elite]
        elite_x = [item[1] for item in elite_pairs]
        elite_mean = _vector_mean(elite_x)
        worst_idx = p_fit.index(max(p_fit))
        worse = p_x[worst_idx]

        w = max(0.7 - 0.5 * ((t + 1) / iterations), 0.3)
        guidance = _vector_add(_vector_mul(best_x, w), _vector_mul(elite_mean, 1 - w))

        # Producers
        r2 = random.random()
        for i in range(p_num):
            if r2 < 0.9:
                a = 1 if random.random() > 0.1 else -1
                term1 = _vector_mul(_vector_abs(_vector_sub(p_x[i], worse)), 0.3)
                term2 = _vector_mul(p_x[i], 0.1 * a)
                x[i] = _vector_add(p_x[i], _vector_add(term1, term2))
            else:
                theta = random.randint(1, 180) * math.pi / 180
                term = _vector_mul(_vector_abs(_vector_sub(p_x[i], p_x[i])), math.tan(theta))
                x[i] = _vector_add(p_x[i], term)
            x[i] = _vector_bounds(x[i], lb, ub)
            fit[i] = objective(x[i])

        # Group updates guided by elites
        r_val = 1 - (t + 1) / iterations
        xnew1 = _vector_bounds(_vector_mul(guidance, 1 - r_val), lb, ub)
        xnew2 = _vector_bounds(_vector_mul(guidance, 1 + r_val), lb, ub)
        for i in range(p_num, population):
            weight = (i) / max(1, (population - 1))
            diff1 = _vector_mul(_vector_sub(p_x[i], xnew1), random.random())
            diff2 = _vector_mul(_vector_sub(p_x[i], xnew2), random.random())
            x[i] = _vector_add(_vector_mul(guidance, weight), _vector_add(diff1, diff2))
            x[i] = _vector_bounds_between(x[i], xnew1, xnew2)
            fit[i] = objective(x[i])

        # Update personal and global bests
        for i in range(population):
            if fit[i] < p_fit[i]:
                p_fit[i] = fit[i]
                p_x[i] = x[i].copy()
            if p_fit[i] < f_min:
                f_min = p_fit[i]
                best_x = p_x[i].copy()

        # Lévy perturbation
        if t + 1 <= round(0.2 * iterations):
            levy_prob = 0.7
            levy_scale = 0.25
        else:
            levy_prob = 0.3 + 0.4 * ((t + 1) / iterations)
            levy_scale = 0.14 + 0.12 * ((t + 1) / iterations)
        beta = 1.5
        sigma_levy = (
            math.gamma(1 + beta)
            * math.sin(math.pi * beta / 2)
            / (math.gamma((1 + beta) / 2) * beta * 2 ** ((beta - 1) / 2))
        ) ** (1 / beta)

        for i in range(population):
            if random.random() < levy_prob:
                u = [random.gauss(0, sigma_levy) for _ in range(dim)]
                v = [random.gauss(0, 1) for _ in range(dim)]
                step = [uu / (abs(vv) ** (1 / beta) + 1e-12) for uu, vv in zip(u, v)]
                x_levy = _vector_add(p_x[i], _vector_mul(step, levy_scale))
                x_levy = _vector_bounds(x_levy, lb, ub)
                fit_levy = objective(x_levy)
                if fit_levy < p_fit[i]:
                    p_fit[i] = fit_levy
                    p_x[i] = x_levy
                if fit_levy < f_min:
                    f_min = fit_levy
                    best_x = x_levy

        # Early adaptive mutation
        for i in range(population):
            if random.random() < 0.1:
                noise = [random.gauss(0, 0.01) * val for val in p_x[i]]
                x_mut = _vector_add(p_x[i], noise)
                x_mut = _vector_bounds(x_mut, lb, ub)
                fit_mut = objective(x_mut)
                if fit_mut < p_fit[i]:
                    p_fit[i] = fit_mut
                    p_x[i] = x_mut
                if fit_mut < f_min:
                    f_min = fit_mut
                    best_x = x_mut

        stagnation_window = 3
        if t + 1 > stagnation_window:
            window_start = max(0, t - stagnation_window)
            recent = convergence[window_start : t + 1]
            if all(abs(val - f_min) < 1e-8 for val in recent if val is not math.inf):
                for i in range(population):
                    if random.random() < 0.7:
                        noise = [random.gauss(0, 1) * val for val in p_x[i]]
                        x_mut = _vector_add(p_x[i], noise)
                    else:
                        x_mut = _random_vector(lb, ub)
                    x_mut = _vector_bounds(x_mut, lb, ub)
                    fit_mut = objective(x_mut)
                    if fit_mut < p_fit[i]:
                        p_fit[i] = fit_mut
                        p_x[i] = x_mut
                    if fit_mut < f_min:
                        f_min = fit_mut
                        best_x = x_mut

        # Predator avoidance
        danger_k = 5
        danger_var_th = 1e-6
        danger_ratio = 0.2
        if (t + 1) > danger_k:
            prev_idx = max(0, t - danger_k)
            stagnated = convergence[prev_idx] == f_min
            low_var = pvariance(p_fit) < danger_var_th if len(p_fit) > 1 else False
            if stagnated or low_var:
                num_danger = max(1, round(danger_ratio * population))
                worst_indices = sorted(range(population), key=lambda idx: p_fit[idx], reverse=True)[:num_danger]
                for idx in worst_indices:
                    p_x[idx] = _random_vector(lb, ub)
                    p_fit[idx] = objective(p_x[idx])
                    if p_fit[idx] < f_min:
                        f_min = p_fit[idx]
                        best_x = p_x[idx]

        convergence[t] = f_min

    return f_min, best_x, convergence


# ---------------------------------------------------------------------------
# Convenience wrapper for trajectory planning
# ---------------------------------------------------------------------------

@dataclass
class MidboResult:
    best_cost: Number
    best_path: List[Tuple[Number, Number, Number]]
    convergence: List[Number]


def run_midbo_planner(env: TrajectoryEnvironment, params: MidboParameters) -> MidboResult:
    dim = 3 * env.n_waypoints
    if params.lower_bounds is None or params.upper_bounds is None:
        lb = [0.0] * dim
        ub = (
            [env.map_range[0]] * env.n_waypoints
            + [env.map_range[1]] * env.n_waypoints
            + [env.map_range[2]] * env.n_waypoints
        )
    else:
        lb = list(params.lower_bounds)
        ub = list(params.upper_bounds)

    best_cost, best_vector, convergence = midbo(
        params.population,
        params.iterations,
        lb,
        ub,
        dim,
        env.cost,
        params.random_seed,
    )
    best_path = env._sample_path(best_vector)
    return MidboResult(best_cost=best_cost, best_path=best_path, convergence=convergence)


# ---------------------------------------------------------------------------
# Pre-set obstacle field matching Get_Functions_details.m (F2)
# ---------------------------------------------------------------------------

PRESET_OBSTACLES_F2: List[Tuple[Number, Number, Number, Number, Number, Number]] = [
    (15, 20, 0, 10, 12, 60),
    (35, 25, 0, 12, 10, 80),
    (55, 30, 0, 14, 10, 90),
    (75, 20, 0, 10, 14, 70),
    (20, 55, 0, 12, 12, 85),
    (45, 65, 0, 16, 10, 60),
    (70, 55, 0, 12, 16, 95),
    (85, 40, 0, 10, 10, 50),
    (30, 80, 0, 14, 12, 70),
    (55, 85, 0, 12, 14, 80),
    (80, 75, 0, 10, 12, 65),
    (10, 35, 0, 10, 10, 55),
    (90, 60, 0, 10, 10, 60),
]


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def _run_cli() -> None:
    parser = argparse.ArgumentParser(description="Run the MIDBO trajectory planner (Python port)")
    parser.add_argument("--iterations", type=int, default=30, help="Number of iterations")
    parser.add_argument("--population", type=int, default=20, help="Population size")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducibility")
    args = parser.parse_args()

    env = TrajectoryEnvironment(
        start_pos=(10, 10, 10),
        goal_pos=(175, 175, 50),
        map_range=(200, 200, 200),
        boxes=PRESET_OBSTACLES_F2,
        n_waypoints=2,
    )
    params = MidboParameters(
        population=args.population,
        iterations=args.iterations,
        random_seed=args.seed,
    )
    result = run_midbo_planner(env, params)

    print("Best cost:", result.best_cost)
    print("Best path (first 5 points):")
    for pt in result.best_path[:5]:
        print("  ", tuple(round(v, 3) for v in pt))
    print("Convergence (first 10 values):", [round(v, 3) for v in result.convergence[:10]])


if __name__ == "__main__":
    _run_cli()
