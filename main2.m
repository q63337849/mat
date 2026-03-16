close all
clear
clc
warning off;

%% 二维栅格路径规划模型（10x10）
global startPos goalPos N staticObstacleCount circles
N = 2;                           % 中间控制点个数（可调）
startPos = [1, 1];               % 起点（可调）
goalPos  = [9, 9];               % 终点（可调）
staticObstacleCount = 8;         % 静态圆形障碍物数量（可自定义）

SearchAgents_no = 30;            % 种群规模（可调）
Function_name   = 'F2';          % F1: 随机圆形障碍; F2: 固定圆形障碍（默认F2避免直线路径过于平凡）
Max_iteration   = 200;           % 最大迭代次数（可调）
RunTimes        = 30;            % 独立运行次数
rng(2024);                       % 固定随机种子，保证可复现

% 加载场景与目标函数
[lb,ub,dim,fobj] = Get_Functions_details(Function_name);
AlgorithmName = {'MIDBO','DBO','WOA','GWO'};
addpath('./AlgorithmCode/');

result = struct([]);
L_straight = norm(goalPos - startPos);

for i = 1:numel(AlgorithmName)
    Algorithm = str2func(AlgorithmName{i});

    J_values = zeros(RunTimes,1);
    t_values = zeros(RunTimes,1);
    bestPosRuns = zeros(RunTimes, dim);

    for k = 1:RunTimes
        tic;
        [Best_score,Best_pos,~] = Algorithm(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
        t_values(k) = toc;

        J_values(k) = Best_score;
        bestPosRuns(k,:) = Best_pos;
    end

    validMask = isfinite(J_values);
    validCount = sum(validMask);
    invalidCount = RunTimes - validCount;

    if validCount > 0
        J_valid = J_values(validMask);
        pos_valid = bestPosRuns(validMask,:);

        [J_best, idxBest] = min(J_valid);
        bestPos = pos_valid(idxBest,:);
        J_mean = mean(J_valid);
        J_std = std(J_valid);
        successRate = 100 * validCount / RunTimes;

        pathXY = build_path(bestPos, N, startPos, goalPos, 120);
        [L, d_min, turnCount, avgTurnDeg] = path_quality_metrics(pathXY, circles);

        L_runs = zeros(validCount,1);
        dmin_runs = zeros(validCount,1);
        turn_runs = zeros(validCount,1);
        avgturn_runs = zeros(validCount,1);
        for vr = 1:validCount
            path_vr = build_path(pos_valid(vr,:), N, startPos, goalPos, 120);
            [L_runs(vr), dmin_runs(vr), turn_runs(vr), avgturn_runs(vr)] = path_quality_metrics(path_vr, circles);
        end
    else
        J_best = NaN;
        J_mean = NaN;
        J_std = NaN;
        successRate = 0;
        bestPos = NaN(1, dim);
        pathXY = NaN(120, 2);

        L = NaN;
        d_min = NaN;
        turnCount = NaN;
        avgTurnDeg = NaN;
        L_runs = NaN;
        dmin_runs = NaN;
        turn_runs = NaN;
        avgturn_runs = NaN;
    end

    result(i).Algorithm = AlgorithmName{i};
    result(i).L = L;
    result(i).d_min = d_min;
    result(i).turn_count = turnCount;
    result(i).avg_turn_deg = avgTurnDeg;
    result(i).L_mean = mean(L_runs);
    result(i).L_std = std(L_runs);
    result(i).d_min_mean = mean(dmin_runs);
    result(i).d_min_std = std(dmin_runs);
    result(i).turn_count_mean = mean(turn_runs);
    result(i).avg_turn_deg_mean = mean(avgturn_runs);
    result(i).detour_ratio = safe_ratio(L, L_straight);

    result(i).J_best = J_best;
    result(i).J_mean = J_mean;
    result(i).J_std = J_std;
    result(i).valid_runs = validCount;
    result(i).invalid_runs = invalidCount;
    result(i).success_rate = successRate;

    result(i).t_global_mean = mean(t_values);
    result(i).t_global_std = std(t_values);
    result(i).J_cv = safe_ratio(result(i).J_std, result(i).J_mean);
    result(i).t_global_cv = safe_ratio(result(i).t_global_std, result(i).t_global_mean);

    result(i).bestPos = bestPos;
    result(i).bestPath = pathXY;
    result(i).J_all = J_values;
    result(i).t_all = t_values;
    result(i).L_all_valid = L_runs;
    result(i).d_min_all_valid = dmin_runs;
end

%% 输出指标
fprintf('\n================ 指标统计（%d 次独立运行） ================\n', RunTimes);
for i = 1:numel(result)
    fprintf('\n[%s]\n', result(i).Algorithm);
    fprintf('路径质量指标:\n');
    fprintf('  路径总长度 L            = %.4f\n', result(i).L);
    fprintf('  最小安全距离 d_min      = %.4f\n', result(i).d_min);
    fprintf('  拐点数量                = %g\n', result(i).turn_count);
    fprintf('  平均转弯角(度)          = %.4f\n', result(i).avg_turn_deg);
    fprintf('  路径长度 L(mean±std)    = %.4f ± %.4f\n', result(i).L_mean, result(i).L_std);
    fprintf('  最小安全距 d_min(mean)  = %.4f\n', result(i).d_min_mean);
    fprintf('  相对直线绕行率 L/L0     = %.4f  (L0=%.4f)\n', result(i).detour_ratio, L_straight);

    fprintf('优化性能指标:\n');
    fprintf('  综合代价终值 J(best)    = %.6f\n', result(i).J_best);
    fprintf('  综合代价终值 J(mean)    = %.6f\n', result(i).J_mean);
    fprintf('  综合代价终值 J(std)     = %.6f\n', result(i).J_std);
    fprintf('  综合代价变异系数 CV(J)  = %.6f\n', result(i).J_cv);
    fprintf('  有效运行次数            = %d/%d\n', result(i).valid_runs, RunTimes);
    fprintf('  失效运行次数            = %d\n', result(i).invalid_runs);
    fprintf('  规划成功率              = %.2f%%\n', result(i).success_rate);
    fprintf('  规划耗时 t_global(mean) = %.6f s\n', result(i).t_global_mean);
    fprintf('  规划耗时 t_global(std)  = %.6f s\n', result(i).t_global_std);
    fprintf('  耗时变异系数 CV(t)      = %.6f\n', result(i).t_global_cv);
end
fprintf('\n============================================================\n');

% 排名分析（越小越好）
J_mean_all = [result.J_mean];
t_mean_all = [result.t_global_mean];
[~, idxJ] = sort(J_mean_all, 'ascend');
[~, idxt] = sort(t_mean_all, 'ascend');
fprintf('\nJ(mean) 排名（越小越好）: ');
for r = 1:numel(idxJ)
    fprintf('%d)%s ', r, result(idxJ(r)).Algorithm);
end
fprintf('\n耗时 t_global(mean) 排名（越小越好）: ');
for r = 1:numel(idxt)
    fprintf('%d)%s ', r, result(idxt(r)).Algorithm);
end
fprintf('\n');

if all([result.detour_ratio] < 1.05)
    fprintf('提示: 所有算法的 L/L0 < 1.05，场景可能过于简单（接近直线路径）。可切换更复杂障碍配置。\n');
end

save('main2_metrics.mat', 'result');


function pathXY = build_path(bestPos, N, startPos, goalPos, nInterp)
x_seq = [startPos(1), bestPos(1:N), goalPos(1)];
y_seq = [startPos(2), bestPos(N+1:2*N), goalPos(2)];
k = length(x_seq);
I_seq = linspace(0,1,nInterp);
X_seq = spline(linspace(0,1,k), x_seq, I_seq);
Y_seq = spline(linspace(0,1,k), y_seq, I_seq);
pathXY = [X_seq(:), Y_seq(:)];
end

function [L, d_min, turnCount, avgTurnDeg] = path_quality_metrics(pathXY, circles)
% 路径总长度
seg = diff(pathXY, 1, 1);
L = sum(sqrt(sum(seg.^2, 2)));

% 最小安全距离（路径点到最近障碍物边界距离）
if isempty(circles)
    d_min = inf;
else
    d_min = inf;
    for p = 1:size(pathXY,1)
        dToCircles = sqrt(sum((circles(:,1:2) - pathXY(p,:)).^2, 2)) - circles(:,3);
        d_min = min(d_min, min(dToCircles));
    end
end

% 转角统计
anglesDeg = [];
for i = 2:size(pathXY,1)-1
    v1 = pathXY(i,:) - pathXY(i-1,:);
    v2 = pathXY(i+1,:) - pathXY(i,:);
    n1 = norm(v1);
    n2 = norm(v2);
    if n1 < 1e-10 || n2 < 1e-10
        continue;
    end
    cosTheta = dot(v1,v2) / (n1*n2);
    cosTheta = max(-1, min(1, cosTheta));
    theta = acosd(cosTheta);
    anglesDeg(end+1,1) = theta; %#ok<AGROW>
end

turnThresholdDeg = 5; % 小于该角度视为近似直行

if isempty(anglesDeg)
    turnCount = 0;
    avgTurnDeg = 0;
else
    turnAngles = anglesDeg(anglesDeg > turnThresholdDeg);
    turnCount = numel(turnAngles);
    if turnCount == 0
        avgTurnDeg = 0;
    else
        avgTurnDeg = mean(turnAngles);
    end
end
end

function out = safe_ratio(num, den)
if ~isfinite(num) || ~isfinite(den) || abs(den) < eps
    out = NaN;
else
    out = num / den;
end
end
