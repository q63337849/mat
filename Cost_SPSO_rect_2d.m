function fitness = Cost_SPSO_rect_2d(x)
% 二维路径代价函数（使用有限大惩罚，避免全局Inf导致算法失效）
% F = b1*长度 + b2*障碍威胁 + b3*平滑度 + 约束惩罚

global N startPos goalPos mapRange boxes

bk = [5, 1, 1];
D = 0.35;            % 小地图下减小硬碰撞膨胀
S = 1.8;             % 小地图下减小软惩罚距离
a1 = 1.0;
nsample = 120;

penaltyOut = 2e4;    % 越界惩罚
penaltyHit = 8e3;    % 碰撞惩罚

x_seq = [startPos(1), x(1:N),       goalPos(1)];
y_seq = [startPos(2), x(N+1:2*N),   goalPos(2)];
k = numel(x_seq);
i_seq = linspace(0,1,k);
I_seq = linspace(0,1,nsample);
X_seq = spline(i_seq, x_seq, I_seq);
Y_seq = spline(i_seq, y_seq, I_seq);
P = [X_seq(:), Y_seq(:)];

% F1: 路径长度
seg = diff(P,1,1);
F1 = sum(sqrt(sum(seg.^2,2)));

% 边界惩罚（有限大）
outMask = (P(:,1) < 0) | (P(:,1) > mapRange(1)) | (P(:,2) < 0) | (P(:,2) > mapRange(2));
Fbound = penaltyOut * nnz(outMask);

% F2: 威胁代价（二维矩形）
F2 = 0;
hitCount = 0;
for j = 1:size(P,1)-1
    p0 = P(j,:); p1 = P(j+1,:);
    for kbox = 1:size(boxes,1)
        b = boxes(kbox,:);
        rect = [b(1), b(2), b(4), b(5)];
        hardRect = inflateRect(rect, D);

        if segmentRectIntersect(p0,p1,hardRect)
            hitCount = hitCount + 1;
            F2 = F2 + penaltyHit;
            continue;
        end

        dk = segmentRectDistance(p0,p1,hardRect);
        if dk < S
            F2 = F2 + (S - dk);
        end
    end
end

% F3: 平滑度（二维转角）
phi = zeros(max(0,size(P,1)-2),1);
for j = 1:size(P,1)-2
    v1 = P(j+1,:) - P(j,:);
    v2 = P(j+2,:) - P(j+1,:);
    nrm = norm(v1)*norm(v2);
    if nrm < 1e-12
        phi(j) = 0;
    else
        c = max(-1,min(1,dot(v1,v2)/nrm));
        phi(j) = acos(c);
    end
end
F3 = a1*sum(abs(phi));

fitness = bk(1)*F1 + bk(2)*F2 + bk(3)*F3 + Fbound;

% 兜底：保证返回有限值
if ~isfinite(fitness)
    fitness = 1e9 + 1e5*hitCount + 1e5*nnz(outMask);
end
end

function r2 = inflateRect(r, d)
r2 = [r(1)-d, r(2)-d, r(3)+2*d, r(4)+2*d];
end

function tf = segmentRectIntersect(p0,p1,r)
xmin=r(1); ymin=r(2); xmax=r(1)+r(3); ymax=r(2)+r(4);
if inRect(p0,r) || inRect(p1,r)
    tf = true; return;
end
e = [xmin ymin xmax ymin; xmax ymin xmax ymax; xmax ymax xmin ymax; xmin ymax xmin ymin];
tf = false;
for i=1:4
    if segSegIntersect(p0,p1,e(i,1:2),e(i,3:4))
        tf = true; return;
    end
end
end

function tf = inRect(p,r)
tf = (p(1)>=r(1) && p(1)<=r(1)+r(3) && p(2)>=r(2) && p(2)<=r(2)+r(4));
end

function tf = segSegIntersect(a,b,c,d)
ccw = @(p1,p2,p3) (p3(2)-p1(2))*(p2(1)-p1(1)) > (p2(2)-p1(2))*(p3(1)-p1(1));
tf = (ccw(a,c,d) ~= ccw(b,c,d)) && (ccw(a,b,c) ~= ccw(a,b,d));
end

function d = segmentRectDistance(p0,p1,r)
T = linspace(0,1,25);
dmin = inf;
for t = T
    p = p0 + t*(p1-p0);
    qx = min(max(p(1), r(1)), r(1)+r(3));
    qy = min(max(p(2), r(2)), r(2)+r(4));
    dmin = min(dmin, norm([p(1)-qx, p(2)-qy]));
end
d = dmin;
end
