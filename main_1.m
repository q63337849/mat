close all
clear
clc
warning off;

%% 三维路径规划模型定义
global startPos goalPos N boxes
N = 2;                                                     % 待优化点的个数(可以修改)
startPos = [10, 10, 10];                                   % 起点(可以修改)
goalPos = [175, 175, 50];                                  % 终点(可以修改)
SearchAgents_no = 30;                                      % 种群大小(可以修改)
Function_name = 'F1';                                      % F1:随机地图 F2:固定地图
Max_iteration = 200;                                       % 最大迭代次数(可以修改)
numRuns = 30;                                              % 独立运行次数

% 获取函数细节
[lb, ub, dim, fobj] = Get_Functions_details(Function_name);

% 强制确保边界适合航迹规划（非负坐标）
fprintf('原始问题边界: lb=[%s], ub=[%s]\n', num2str(lb), num2str(ub));
lb = max(lb, 0);
if any(ub <= lb)
    ub = max(ub, max(goalPos) + 50);
end
fprintf('修正后边界: lb=[%s], ub=[%s]\n', num2str(lb), num2str(ub));

% 算法列表
AlgorithmName = {'MIDBO', 'DBO', 'WOA', 'GWO'};
addpath('./AlgorithmCode/');

bestFit = nan(1, numel(AlgorithmName));
data = struct();
metricsSummary = struct();

for i = 1:numel(AlgorithmName)
    fprintf('\n=== 开始运行算法: %s (%d次独立运行) ===\n', AlgorithmName{i}, numRuns);
    Algorithm = str2func(AlgorithmName{i});

    J_runs = nan(1, numRuns);
    t_runs = nan(1, numRuns);
    bestScore = inf;
    bestPos = [];
    bestCurve = [];

    for runIdx = 1:numRuns
        try
            tStart = tic;
            [Best_score, Best_pos, Convergence_curve] = Algorithm(SearchAgents_no, Max_iteration, lb, ub, dim, fobj);
            tCost = toc(tStart);

            % 修正非法值
            if any(Best_pos < 0)
                Best_pos = max(Best_pos, 0);
            end

            J_runs(runIdx) = Best_score;
            t_runs(runIdx) = tCost;

            if isfinite(Best_score) && Best_score < bestScore
                bestScore = Best_score;
                bestPos = Best_pos;
                bestCurve = Convergence_curve;
            end

            fprintf('[%s][Run %02d/%02d] J=%.6f, t_global=%.4fs\n', ...
                AlgorithmName{i}, runIdx, numRuns, Best_score, tCost);
        catch ME
            fprintf('[%s][Run %02d/%02d] 运行失败: %s\n', AlgorithmName{i}, runIdx, numRuns, ME.message);
        end
    end

    validJ = J_runs(isfinite(J_runs));
    validT = t_runs(isfinite(t_runs));

    if isempty(validJ)
        J_best = inf;
        J_mean = inf;
        J_std = inf;
    else
        J_best = min(validJ);
        J_mean = mean(validJ);
        J_std = std(validJ);
    end

    if isempty(validT)
        t_global = inf;
    else
        t_global = validT(1); % 单次规划耗时（取首次成功运行）
    end

    pathMetrics = evaluate_path_metrics(bestPos, N, startPos, goalPos, boxes);

    data(i).Best_score = bestScore;
    data(i).Best_pos = bestPos;
    data(i).Convergence_curve = bestCurve;
    data(i).J_runs = J_runs;
    data(i).t_runs = t_runs;

    metricsSummary(i).Algorithm = AlgorithmName{i};
    metricsSummary(i).L = pathMetrics.L;
    metricsSummary(i).d_min = pathMetrics.d_min;
    metricsSummary(i).turning_count = pathMetrics.turning_count;
    metricsSummary(i).avg_turn_angle_deg = pathMetrics.avg_turn_angle_deg;
    metricsSummary(i).J_best_30 = J_best;
    metricsSummary(i).J_mean_30 = J_mean;
    metricsSummary(i).J_std_30 = J_std;
    metricsSummary(i).t_global = t_global;

    bestFit(i) = bestScore;

    fprintf('--- %s 指标汇总 ---\n', AlgorithmName{i});
    fprintf('路径总长度 L = %.4f\n', pathMetrics.L);
    fprintf('最小安全距离 d_min = %.4f\n', pathMetrics.d_min);
    fprintf('拐点数量 = %d, 平均转弯角 = %.4f°\n', pathMetrics.turning_count, pathMetrics.avg_turn_angle_deg);
    fprintf('综合代价 J(30次): 最优=%.6f, 平均=%.6f, 标准差=%.6f\n', J_best, J_mean, J_std);
    fprintf('规划耗时 t_global(单次) = %.4fs\n', t_global);
end

fprintf('\n=== 最终指标对比表 ===\n');
for i = 1:numel(metricsSummary)
    fprintf(['%s | L=%.4f | d_min=%.4f | 拐点=%d | 平均转角=%.4f° | ', ...
             'J(best/mean/std)=%.6f/%.6f/%.6f | t_global=%.4fs\n'], ...
             metricsSummary(i).Algorithm, metricsSummary(i).L, metricsSummary(i).d_min, ...
             metricsSummary(i).turning_count, metricsSummary(i).avg_turn_angle_deg, ...
             metricsSummary(i).J_best_30, metricsSummary(i).J_mean_30, metricsSummary(i).J_std_30, ...
             metricsSummary(i).t_global);
end

% 保存数据
save('data.mat', 'data', 'metricsSummary');

%% 绘制结果图
if ~exist('./Picture', 'dir')
    mkdir('./Picture');
end

% 直方图（最佳J）
figure
validMask = isfinite(bestFit);
validFit = bestFit(validMask);
validNames = AlgorithmName(validMask);
if ~isempty(validFit)
    bar(validFit)
    ylabel('适应度(最优J)');
    set(gca, 'xtick', 1:length(validNames));
    set(gca, 'XTickLabel', validNames);
    title('各算法最优J对比');
    grid on;
else
    text(0.5, 0.5, '所有算法均失败', 'HorizontalAlignment', 'center');
end
set(gcf, 'color', 'w');
saveas(gcf, './Picture/直方图.jpg');

% 收敛曲线
strColor = {'r-', 'g-', 'b-', 'k-', 'm-', 'c-', 'y-'};
figure
legendEntries = {};
plotCount = 0;
for i = 1:numel(data)
    if isfield(data(i), 'Convergence_curve') && ~isempty(data(i).Convergence_curve) && all(isfinite(data(i).Convergence_curve))
        plotCount = plotCount + 1;
        plot(data(i).Convergence_curve, strColor{mod(i-1, length(strColor)) + 1}, 'LineWidth', 1.5);
        hold on;
        legendEntries{plotCount} = AlgorithmName{i}; %#ok<SAGROW>
    end
end
if plotCount > 0
    xlabel('迭代次数');
    ylabel('代价值 J');
    legend(legendEntries, 'Location', 'Best');
    title('算法收敛曲线对比');
    grid on;
else
    text(0.5, 0.5, '没有有效的收敛数据', 'HorizontalAlignment', 'center');
end
set(gcf, 'color', 'w');
saveas(gcf, './Picture/收敛曲线.jpg');

%% 显示三维图
try
    set(0, 'DefaultFigureVisible', 'on');
    path_pts = plotFigure_rect(data, AlgorithmName, strColor);
    hFig3 = gcf;
    ax3 = gca;
    view(ax3, 3);
    axis(ax3, 'equal');
    drawnow;
    shg;
    saveas(hFig3, './Picture/路径曲线（三维）.jpg');

    % 二维图
    hFig2 = figure('Visible', 'off', 'Name', '二维快照', 'NumberTitle', 'off');
    ax2 = copyobj(ax3, hFig2);
    set(ax2, 'Units', 'normalized', 'Position', [0.13 0.11 0.775 0.815]);
    view(ax2, 2);
    axis(ax2, 'equal');
    drawnow;
    saveas(hFig2, './Picture/路径曲线（二维）.jpg');
    close(hFig2);

    figure(hFig3);
    drawnow;
    shg;

    save('path_data.mat', 'path_pts');
catch ME
    fprintf('绘制路径图时出错: %s\n', ME.message);
end

fprintf('\n程序执行完成！\n');
