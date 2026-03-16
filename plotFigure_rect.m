function path=plotFigure_rect(data,LegendStr,strcolor)
% 二维绘图（函数名保持不变，兼容原调用）
global N startPos goalPos boxes mapRange
num = numel(data);

for i=1:num
    x = data(i).Best_pos;
    if isempty(x) || any(~isfinite(x)) || numel(x) < 2*N
        path(i).data = nan(100,2); %#ok<AGROW>
        continue;
    end
    x_seq = [startPos(1), x(1:N),     goalPos(1)];
    y_seq = [startPos(2), x(N+1:2*N), goalPos(2)];
    k = length(x_seq);
    I_seq = linspace(0,1,200);
    X_seq = spline(linspace(0,1,k), x_seq, I_seq);
    Y_seq = spline(linspace(0,1,k), y_seq, I_seq);
    path(i).data = [X_seq', Y_seq']; %#ok<AGROW>
end

figure; hold on; box on; grid on
axis([0 mapRange(1) 0 mapRange(2)])
axis equal
xlabel('x'); ylabel('y');

if isempty(boxes)
    warning('当前场景未生成任何障碍物（boxes为空）。');
else
    for j=1:size(boxes,1)
        rectangle('Position',[boxes(j,1), boxes(j,2), boxes(j,4), boxes(j,5)], ...
                  'FaceColor',[0.90 0.25 0.25 0.65], 'EdgeColor',[0.2 0.2 0.2], 'LineWidth',0.8);
    end
end

scatter(startPos(1), startPos(2), 70, 'g', 'filled');
text(startPos(1), startPos(2)+0.6, '起点');
scatter(goalPos(1), goalPos(2), 70, 'b', 'filled');
text(goalPos(1), goalPos(2)+0.6, '终点');

leg = gobjects(1,num);
nameList = {};
k = 0;
for i=1:num
    if all(isnan(path(i).data(:)))
        continue;
    end
    k = k + 1;
    leg(k) = plot(path(i).data(:,1), path(i).data(:,2), strcolor{i}, 'LineWidth', 2.0);
    nameList{k} = LegendStr{i}; %#ok<AGROW>
end

if k > 0
    legend(leg(1:k), nameList, 'location','best');
end
title(sprintf('二维路径与障碍物（障碍数量: %d）', size(boxes,1)));
set(gcf,'color','w')
end
