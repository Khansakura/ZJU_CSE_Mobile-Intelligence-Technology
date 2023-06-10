function map = draw_wholemap(Rf_remain, Tf_remain, robot_traj)
%本函数的功能是绘制出所有点云对应位姿后叠加形成的局部点云地图及机器人路径
figure('Name','叠加局部点云图及移动路径');
axis equal 
plot3(0, 0, 0, 'y');
set(gcf,'color','black'); %窗口背景黑色
colordef black; %2D/3D图背景黑色
grid on
hold on
%绘制机器人移动路径
for i = 2:1:10
    robot_x = [robot_traj{i}(1),robot_traj{i-1}(1)];
    robot_y = [robot_traj{i}(2),robot_traj{i-1}(2)];
    robot_z = [robot_traj{i}(3),robot_traj{i-1}(3)];
    plot3(robot_x,robot_y,robot_z,'y');
    hold on;
end

%绘制对应位姿的局部点云图
%首先绘制第零帧
matrix_0 = transform('0.ply');
data_0 = matrix_0';
x0=data_0(1,:);
y0=data_0(2,:);
z0=data_0(3,:);
plot3(x0,y0,z0,'r.');
hold on
%绘制叠加后方位姿的点云图
for i=2:1:10
    matrix = transform(strcat(num2str(i-1),'.ply'));
    data_source_2=matrix';
    for j=1:1:i-1
        data_source_2=Rf_remain{i-j}*data_source_2+Tf_remain{i-j};
    end
    x=data_source_2(1,:);
    y=data_source_2(2,:);
    z=data_source_2(3,:);
    plot3(x,y,z,'r.');
    hold on
end
axis equal 



