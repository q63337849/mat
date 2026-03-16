function [lb,ub,dim,fobj] = Get_Functions_details(F)
global N mapRange circles startPos goalPos staticObstacleCount

mapRange = [10,10];   % 二维栅格地图长、宽

switch F
    case 'F1'  % 随机圆形障碍环境
        if isempty(staticObstacleCount)
            staticObstacleCount = 8;
        end
        radiusRange = [0.4, 1.2];
        safeRadius  = 1.2;   % 起点/终点附近无障碍的安全半径
        circles = generate_random_circles(staticObstacleCount, mapRange, startPos, goalPos, radiusRange, safeRadius);

    case 'F2'  % 固定圆形障碍（示例）
        circles = [ ...
            2.5, 4.0, 0.8;
            4.0, 2.0, 0.7;
            5.5, 5.5, 0.9;
            7.0, 3.8, 0.8;
            3.5, 7.2, 0.7;
            6.8, 7.0, 1.0];

    otherwise
        error('未知场景 F');
end

dim = 2*N;
lb  = zeros(1,dim);
ub  = ones(1,dim);
ub(1:N)       = mapRange(1);   % X
ub(N+1:2*N)   = mapRange(2);   % Y

fobj = @Cost_SPSO_rect;
end

function circles = generate_random_circles(K, mapRange, startPos, goalPos, radiusRange, safeRadius)
% circles: [cx cy r]
circles = zeros(K,3);
count = 0;
maxTrials = max(500, 100*K);
trial = 0;

while count < K && trial < maxTrials
    trial = trial + 1;
    r = radiusRange(1) + (radiusRange(2)-radiusRange(1))*rand;
    cx = r + (mapRange(1)-2*r)*rand;
    cy = r + (mapRange(2)-2*r)*rand;

    % 与起点/终点保持安全距离
    if norm([cx,cy] - startPos) <= (r + safeRadius)
        continue;
    end
    if norm([cx,cy] - goalPos) <= (r + safeRadius)
        continue;
    end

    % 避免障碍重叠（可放宽）
    ok = true;
    for i = 1:count
        if norm([cx,cy] - circles(i,1:2)) < (r + circles(i,3) + 0.15)
            ok = false;
            break;
        end
    end
    if ~ok
        continue;
    end

    count = count + 1;
    circles(count,:) = [cx, cy, r];
end

if count < K
    warning('随机障碍仅生成 %d/%d 个（地图拥挤或约束过严）', count, K);
    circles = circles(1:count,:);
end
end
