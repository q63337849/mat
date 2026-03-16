close all
clear
clc
warning off;

%% 二维栅格路径规划模型（10x10）
global startPos goalPos N staticObstacleCount
N = 2;                           % 中间控制点个数（可调）
startPos = [1, 1];               % 起点（可调）
goalPos  = [9, 9];               % 终点（可调）
staticObstacleCount = 8;         % 静态圆形障碍物数量（可自定义）

SearchAgents_no = 30;            % 种群规模（可调）
Function_name   = 'F1';          % F1: 随机圆形障碍; F2: 固定圆形障碍
Max_iteration   = 200;           % 最大迭代次数（可调）

% Load details of the selected benchmark function
[lb,ub,dim,fobj] = Get_Functions_details(Function_name);
AlgorithmName = {'MIDBO','DBO','WOA','GWO'};
addpath('./AlgorithmCode/');

bestFit = [];
for i = 1:size(AlgorithmName,2)
    Algorithm = str2func(AlgorithmName{i});
    [Best_score,Best_pos,Convergence_curve] = Algorithm(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
    data(i).Best_score = Best_score;
    data(i).Best_pos = Best_pos;
    data(i).Convergence_curve = Convergence_curve;
    bestFit = [bestFit data(i).Best_score];
end

disp('bestFit:');
disp(bestFit);
for i = 1:size(data,2)
    disp(['算法 ', AlgorithmName{i}, ' 最优值: ', num2str(data(i).Best_score)]);
end

save data data

%% 柱状图
figure
bar(bestFit)
ylabel('适应值');
set(gca,'xtick',1:1:size(AlgorithmName,2));
set(gca,'XTickLabel',AlgorithmName)
set(gcf,'color','w')
saveas(gcf,'./Picture/直方图.jpg')

%% 收敛曲线
strColor = {'r-','g-','b-','k-','m-','c-','y-'};
figure
for i = 1:size(data,2)
    plot(data(i).Convergence_curve,strColor{i},'linewidth',1.5)
    hold on
end
xlabel('迭代次数');
ylabel('适应值');
legend(AlgorithmName,'Location','Best')
set(gcf,'color','w')
saveas(gcf,'./Picture/收敛曲线.jpg')

%% 绘制二维路径
set(0,'DefaultFigureVisible','on');
path_pts = plotFigure_rect(data, AlgorithmName, strColor);
if ~exist('./Picture','dir'); mkdir('./Picture'); end
saveas(gcf, './Picture/路径曲线（二维）.jpg');
save('path_data.mat','path_pts');
