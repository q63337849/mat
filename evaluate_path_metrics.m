function metrics = evaluate_path_metrics(bestPos, N, startPos, goalPos, boxes)
% 二维路径质量指标：总长度、最小安全距离、拐点数、平均转弯角

if isempty(bestPos) || any(~isfinite(bestPos))
    metrics = struct('L', inf, 'd_min', -inf, 'turning_count', NaN, ...
        'avg_turn_angle_deg', NaN, 'path', []);
    return;
end

sampleN = 200;
xSeq = [startPos(1), bestPos(1:N), goalPos(1)];
ySeq = [startPos(2), bestPos(N+1:2*N), goalPos(2)];

k = numel(xSeq);
t = linspace(0, 1, k);
tq = linspace(0, 1, sampleN);

X = spline(t, xSeq, tq);
Y = spline(t, ySeq, tq);
P = [X(:), Y(:)];

% 路径总长度 L
seg = diff(P, 1, 1);
metrics.L = sum(sqrt(sum(seg.^2, 2)));

% 最小安全距离 d_min（到最近障碍物矩形边界）
dmin = inf;
for i = 1:size(P, 1)
    p = P(i, :);
    for b = 1:size(boxes, 1)
        d = point_to_rect_distance(p, boxes(b, :));
        dmin = min(dmin, d);
    end
end
metrics.d_min = dmin;

% 拐点数量 & 平均转弯角
angles = [];
for i = 2:size(P, 1)-1
    v1 = P(i, :) - P(i-1, :);
    v2 = P(i+1, :) - P(i, :);
    nrm = norm(v1) * norm(v2);
    if nrm < 1e-12
        continue;
    end
    c = max(-1, min(1, dot(v1, v2) / nrm));
    ang = acos(c);
    angles(end+1) = ang; %#ok<AGROW>
end

if isempty(angles)
    metrics.turning_count = 0;
    metrics.avg_turn_angle_deg = 0;
else
    turnThreshold = deg2rad(5);
    metrics.turning_count = sum(angles > turnThreshold);
    metrics.avg_turn_angle_deg = mean(rad2deg(abs(angles)));
end

metrics.path = P;
end

function d = point_to_rect_distance(p, box)
% box=[x y z w l h]，二维只使用 x,y,w,l
bmin = box(1:2);
bmax = box(1:2) + box(4:5);
q = min(max(p, bmin), bmax);
d = norm(p - q);
end
