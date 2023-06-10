% ICP点云估计：
% 对于点云P的每个点，在另一个点云Q中求取距离它最近的点（理想状态下本应重合的点），计算对应点的欧式距离平方的平均值，
% 然后通过迭代算法，最小化平均值，这样不断更新点云片间的相对位置，达到点云片之间配准对齐的效果
clc, clear all
flag = 1;
%分别处理九帧点云图像，得到九个位姿转换矩阵T
T_final_cal = eye(4,4);
Rf_remain{1} = eye(3,3);
Tf_remain{1} = [0,0,0]';
for i = 1:1:9
    %不同帧数之间处理指示
    %number_flag = i;
    %fprintf("正在处理第%d帧",i-1)
    %读入ply文件并转换为txt，读入矩阵
    str1 = [num2str(i-1),'.ply'];
    str2 = [num2str(i),'.ply'];
    ply1 = pcread(str1);
    ply2 = pcread(str2);
    A = transform(str1);
    B = transform(str2);
    Rf_sum = eye(3,3);    %初始化旋转矩阵
    Tf_sum = [0,0,0]';    %初始化位移矩阵
    data_source = A';
    data_target = B';
    iteration = 0;%迭代次数标记初始化
    T_final = eye(4,4);%每次处理重置并初始化迭代矩阵
    Rf=T_final(1:3,1:3);
    Tf=T_final(1:3,4);
    data_target = Rf*data_target + Tf*ones(1,size(data_target,2)); %初次随机校准
    error = 100;
    error_remain=[];
    %开始迭代
    while error > 0.0001
        %fprintf("正在进行第%d次迭代\n",iteration);
        iteration = iteration + 1;
        for x = 1:size(data_target,2)
            %取三个维度的两坐标差u_a
            data_error(1,:) = data_source(1,:) - data_target(1,x);
            data_error(2,:) = data_source(2,:) - data_target(2,x);
            data_error(3,:) = data_source(3,:) - data_target(3,x);
            %构建最小二乘问题求解误差平方和最小
            data_error_sum = 1/2.*(data_error(1,:).^2 + data_error(2,:).^2 + data_error(3,:).^2);  
            %最小值
            [min_error,min_index] = min(data_error_sum);%获取最小误差点的数值及标签
            data_according(:,x) = data_source(:,min_index);%生成与data_target对应的初始点云矩阵
            error_remain(x) = min_error;%记录本次最小二乘最小值
        end
        %线性代数求解方法
        %计算中心
        data_source_mean = mean(data_according,2);
        data_target_mean = mean(data_target,2);
        %计算每个点去质心坐标
        data_target_1 = data_target - data_target_mean*ones(1,size(data_target,2));
        data_source_1 = data_according - data_source_mean*ones(1,size(data_according,2));
        %svd分解,W = USV.T = 求和qi q.T
        W = data_target_1 * data_source_1';
        [U, S, V] = svd(W);
        %求解R,t
        Rf = V * U';
        Tf = data_source_mean-Rf*data_target_mean;
        %求解优化目标函数最值
        Rf_sum = Rf*Rf_sum;
        Tf_sum = Tf_sum + Tf;
        error = mean(error_remain);
        T0 = [Rf';Tf'];
        T_final = [T0 [0;0;0;1]]*T_final;
        T_final_cal = T_final';
        data_target = Rf*data_target + Tf*ones(1,size(data_target,2)); %旋转和平移
        if iteration > 600
            break
        end
    end
    %保存Rf和Tf
    T_final_log = T_final';
    Tf_remain{i} = T_final_log(1:3,4);
    Rf_remain{i} = T_final_log(1:3,1:3);
    fprintf("误差为：%f\n",error)
    
    strname = strcat(str1,'与',str2,'ICP后点云');
    drawpicture(A,data_target',strname);
end

%计算轨迹
robot_traj = calculate_traj(Rf_remain,Tf_remain);
%绘制轨迹
for i = 2:1:10
    x = [robot_traj{i}(1),robot_traj{i-1}(1)];
    y = [robot_traj{i}(2),robot_traj{i-1}(2)];
    z = [robot_traj{i}(3),robot_traj{i-1}(3)];
    hold on;
    plot3(x,y,z,'y.');
    
end
%绘制叠加局部点云图及移动路径
draw_wholemap(Rf_remain, Tf_remain, robot_traj)

