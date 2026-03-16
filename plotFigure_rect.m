function path=plotFigure_rect(data,LegendStr,strcolor)
global N startPos goalPos boxes mapRange
num = numel(data);

% 生成每条路径
for i=1:num
    x = data(i).Best_pos;
    if isempty(x)
        path(i).data = nan(100,3); %#ok<AGROW>
        continue;
    end
    x_seq = [startPos(1), x(1:N),           goalPos(1)];
    y_seq = [startPos(2), x(N+1:2*N),       goalPos(2)];
    z_seq = [startPos(3), x(2*N+1:3*N),     goalPos(3)];
    k = length(x_seq);
    I_seq = linspace(0,1,100);
    X_seq = spline(linspace(0,1,k), x_seq, I_seq);
    Y_seq = spline(linspace(0,1,k), y_seq, I_seq);
    Z_seq = spline(linspace(0,1,k), z_seq, I_seq);
    path(i).data = [X_seq', Y_seq', Z_seq']; %#ok<AGROW>
end

figure; hold on; box on; grid on
axis([0 mapRange(1) 0 mapRange(2) 0 mapRange(3)])
xlabel('x'); ylabel('y'); zlabel('z'); view(3)

% 画长方体障碍（提高可见性）
if isempty(boxes)
    warning('当前场景未生成任何障碍物（boxes为空）。');
else
    for j=1:size(boxes,1)
        draw_box(boxes(j,:), 0.65, [0.90 0.25 0.25]); % 更高不透明度，红色更醒目
    end
end

% 画起点/终点
scatter3(startPos(1), startPos(2), startPos(3), 70, 'g', 'filled')
text(startPos(1), startPos(2), startPos(3)+6, '起点')
scatter3(goalPos(1), goalPos(2), goalPos(3), 70, 'b', 'filled')
text(goalPos(1), goalPos(2), goalPos(3)+6, '终点')

% 画路径
leg = gobjects(1,num);
for i=1:num
    if all(isnan(path(i).data(:)))
        continue;
    end
    leg(i) = plot3(path(i).data(:,1), path(i).data(:,2), path(i).data(:,3), ...
                   strcolor{i}, 'LineWidth', 2.0);
end
legend(leg, LegendStr, 'location','best')
title(sprintf('路径与障碍物可视化（障碍数量: %d）', size(boxes,1)));
set(gcf,'color','w')
end

function draw_box(b, alphaV, fc)
% b = [x y z w l h]  轴对齐长方体
if nargin < 3, fc = [0.8 0.2 0.2]; end
x0 = b(1); y0 = b(2); z0 = b(3);
x1 = x0 + b(4);
y1 = y0 + b(5);
z1 = z0 + b(6);

V = [ ...
    x0 y0 z0;
    x1 y0 z0;
    x1 y1 z0;
    x0 y1 z0;
    x0 y0 z1;
    x1 y0 z1;
    x1 y1 z1;
    x0 y1 z1];

F = [ ...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];

patch('Vertices',V,'Faces',F, ...
      'FaceColor',fc, 'EdgeColor',[0.15 0.15 0.15], ...
      'LineWidth',0.8, 'FaceAlpha',alphaV);
end
