function fitness = Cost_SPSO_rect(x)
% 2D路径代价函数：路径长度 + 障碍威胁 + 平滑度

global N startPos goalPos mapRange circles

% 参数
wLength = 5;
wThreat = 10;
wSmooth = 1;
safeBuffer = 0.3;
nsample = 120;

% 1) 生成二维样条路径
x_seq = [startPos(1), x(1:N), goalPos(1)];
y_seq = [startPos(2), x(N+1:2*N), goalPos(2)];
k = numel(x_seq);
I_seq = linspace(0,1,nsample);
X_seq = spline(linspace(0,1,k), x_seq, I_seq);
Y_seq = spline(linspace(0,1,k), y_seq, I_seq);
P = [X_seq(:), Y_seq(:)];

% 2) 越界检测
if any(P(:,1) < 0 | P(:,1) > mapRange(1) | P(:,2) < 0 | P(:,2) > mapRange(2))
    fitness = inf;
    return;
end

% 3) 路径长度
V = diff(P,1,1);
segLen = sqrt(sum(V.^2,2));
F1 = sum(segLen);

% 4) 障碍碰撞与威胁代价
F2 = 0;
for i = 1:size(circles,1)
    c = circles(i,1:2);
    r = circles(i,3);
    d = sqrt(sum((P - c).^2,2));

    % 硬碰撞
    if any(d <= r)
        fitness = inf;
        return;
    end

    % 缓冲区线性惩罚
    threatMask = d < (r + safeBuffer);
    F2 = F2 + sum((r + safeBuffer) - d(threatMask));
end

% 5) 平滑度（转角变化）
F3 = 0;
for j = 2:size(P,1)-1
    v1 = P(j,:)   - P(j-1,:);
    v2 = P(j+1,:) - P(j,:);
    nrm = norm(v1)*norm(v2);
    if nrm > 1e-10
        cosTheta = max(-1,min(1,dot(v1,v2)/nrm));
        F3 = F3 + acos(cosTheta);
    end
end

fitness = wLength*F1 + wThreat*F2 + wSmooth*F3;
end
