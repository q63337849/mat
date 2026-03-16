function path = plotFigure_rect(data, LegendStr, strcolor)
global N startPos goalPos circles mapRange
num = numel(data);

for i = 1:num
    x = data(i).Best_pos;
    x_seq = [startPos(1), x(1:N), goalPos(1)];
    y_seq = [startPos(2), x(N+1:2*N), goalPos(2)];
    k = length(x_seq);
    I_seq = linspace(0,1,120);
    X_seq = spline(linspace(0,1,k), x_seq, I_seq);
    Y_seq = spline(linspace(0,1,k), y_seq, I_seq);
    path(i).data = [X_seq', Y_seq'];
end

figure; hold on; box on; grid on
axis([0 mapRange(1) 0 mapRange(2)])
axis equal
xlabel('x'); ylabel('y');
title('10x10二维栅格环境路径规划')
set(gca,'XTick',0:1:mapRange(1),'YTick',0:1:mapRange(2));

% 画圆形障碍物
for j = 1:size(circles,1)
    draw_circle(circles(j,1), circles(j,2), circles(j,3), [0.85 0.2 0.2], 0.35);
end

% 起点终点
scatter(startPos(1), startPos(2), 60, 'g', 'filled')
text(startPos(1)+0.12, startPos(2)+0.12, '起点')
scatter(goalPos(1), goalPos(2), 60, 'b', 'filled')
text(goalPos(1)+0.12, goalPos(2)+0.12, '终点')

% 路径
leg = gobjects(1,num);
for i = 1:num
    leg(i) = plot(path(i).data(:,1), path(i).data(:,2), strcolor{i}, 'LineWidth', 2.0);
end
legend(leg, LegendStr, 'location', 'best')
set(gcf,'color','w')
end

function draw_circle(cx, cy, r, colorV, alphaV)
theta = linspace(0,2*pi,80);
xx = cx + r*cos(theta);
yy = cy + r*sin(theta);
patch(xx, yy, colorV, 'FaceAlpha', alphaV, 'EdgeColor', colorV, 'LineWidth', 1.0);
end
