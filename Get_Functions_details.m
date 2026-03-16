function [lb,ub,dim,fobj] = Get_Functions_details(F)
global N mapRange boxes startPos goalPos

mapRange = [20,20];   % 二维地图范围 [Lx, Ly]

switch F
    case 'F1'  % 随机产生二维障碍（仍沿用长方体结构，实际只用XY投影）
        K = 15;                % 障碍数量（可调）
        minSize = [1.2, 1.2, 1];   % XY最小尺寸（适配20x20地图）
        maxSize = [3.0,3.0,1];   % XY最大尺寸（适配20x20地图）
        minGap  = 0.6;           % 障碍-障碍/障碍-边界 间隙（适配20x20地图）
        boxes = gen_rect_obstacles(K, [mapRange, 1], minSize, maxSize, minGap, startPos, goalPos);

    case 'F2'  % 固定参数（二维投影）
        boxes = [...
            15  20   0   10 12 1;
            35  25   0   12 10 1;
            55  30   0   14 10 1;
            75  20   0   10 14 1;
            20  55   0   12 12 1;
            45  65   0   16 10 1;
            70  55   0   12 16 1;
            85  40   0   10 10 1;
            30  80   0   14 12 1;
            55  85   0   12 14 1;
            80  75   0   10 12 1;
            10  35   0   10 10 1;
            90  60   0   10 10 1  ];
    otherwise
        error('未知场景 F');
end

dim = 2*N;
lb  = zeros(1,dim);
ub  = ones(1,dim);
ub(1:N)           = mapRange(1);   % X
ub(N+1:2*N)       = mapRange(2);   % Y

fobj = @Cost_SPSO_rect_2d;
end
