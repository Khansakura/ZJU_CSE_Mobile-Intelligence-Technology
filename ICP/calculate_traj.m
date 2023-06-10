function robot_traj = calculate_traj(Rf_remain,Tf_remain)
%此函数用于计算轨迹
robot_traj{1}=[0,0,0]';
for n=2:1:10
    robot_traj{n}=[0,0,0]';
    for j=1:1:n-1
        robot_traj{n}=Rf_remain{n-j}*robot_traj{n}+Tf_remain{n-j};
    end
    fprintf('第%d次移动后位置:',n-2);
    fprintf('%f,%f,%f\n',robot_traj{n}(1),robot_traj{n}(2),robot_traj{n}(3));
end

%测试调试
% for n = 2:1:10
%     Tf_final_traj=Tf_remain{n-1};
%     Rf_final_traj=Rf_remain{n-1};
%     robot_traj{n}=Rf_final_traj*robot_traj{n-1}+Tf_final_traj;
% 
% end