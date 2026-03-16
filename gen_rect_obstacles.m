function boxes = gen_rect_obstacles(K, mapRange, minSize, maxSize, minGap, startPos, goalPos)
% 生成K个互不重叠的轴对齐长方体障碍物（AABB）
% mapRange = [Lx, Ly, Lz]
% 每个box格式：[x y z w l h]，原点在box最小角 (x,y,z)，尺寸分别为 w(沿x) l(沿y) h(沿z)
% minGap: 障碍之间以及与边界的最小安全间隙
% startPos/goalPos: 可选，若提供则在XY平面上为起终点预留安全区

if nargin < 6, startPos = []; end
if nargin < 7, goalPos = []; end

boxes = zeros(K,6);
Lx = mapRange(1); Ly = mapRange(2);

sxy = to_xy(startPos);
gxy = to_xy(goalPos);
clearRadius = max(minGap, max(minSize(1:2))); % 起终点周围安全半径

tries = 0; i = 1;
while i <= K
    tries = tries + 1;
    if tries > 5000
        error('放置障碍失败，参数过于苛刻');
    end

    sz = minSize + (maxSize-minSize).*rand(1,3);  % [w l h]
    % 预留与边界的间隙
    x = minGap + (Lx - sz(1) - 2*minGap) * rand;
    y = minGap + (Ly - sz(2) - 2*minGap) * rand;
    z = 0;  % 地面起
    box = [x, y, z, sz(1), sz(2), sz(3)];

    % 与已放置障碍保持 minGap 间隙
    ok = true;
    for j = 1:i-1
        if aabb_overlap_expanded(box, boxes(j,:), minGap)
            ok = false;
            break;
        end
    end

    % 避免障碍贴近起点/终点（仅使用XY，兼容2D/3D位置向量）
    if ok && ~isempty(sxy)
        if point_to_aabb_xy_distance(sxy, box) < clearRadius
            ok = false;
        end
    end

    if ok && ~isempty(gxy)
        if point_to_aabb_xy_distance(gxy, box) < clearRadius
            ok = false;
        end
    end

    if ok
        boxes(i,:) = box;
        i = i + 1;
    end
end
end

function o = aabb_overlap_expanded(b1, b2, gap)
% 判定两个长方体在各方向上是否相交（附加 gap 间隙）
% b = [x y z w l h]
a1 = [b1(1)-gap, b1(2)-gap, b1(3)-gap, b1(4)+2*gap, b1(5)+2*gap, b1(6)+2*gap];
a2 = [b2(1),     b2(2),     b2(3),     b2(4),       b2(5),       b2(6)];
o = ~ ( (a1(1)+a1(4) <= a2(1)) || (a2(1)+a2(4) <= a1(1)) || ...
        (a1(2)+a1(5) <= a2(2)) || (a2(2)+a2(5) <= a1(2)) || ...
        (a1(3)+a1(6) <= a2(3)) || (a2(3)+a2(6) <= a1(3)) );
end

function xy = to_xy(pos)
xy = [];
if isempty(pos)
    return;
end
pos = pos(:)';
if numel(pos) >= 2
    xy = pos(1:2);
elseif numel(pos) == 1
    xy = [pos(1), pos(1)];
end
end

function d = point_to_aabb_xy_distance(pxy, box)
% pxy: [x y], box=[x y z w l h]
bmin = box(1:2);
bmax = box(1:2) + box(4:5);
q = min(max(pxy, bmin), bmax);
d = norm(pxy - q);
end
