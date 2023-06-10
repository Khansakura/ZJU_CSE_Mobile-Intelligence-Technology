function picture = drawpicture(A,B,strname)
%该函数实现了将矩阵数据转化成点云数据并且绘图的功能
    x1 = A(:, 1);y1 = A(:, 2);z1 = A(:, 3);
    x2 = B(:, 1);y2 = B(:, 2);z2 = B(:, 3);
    figure('Name', strname);
    set(gcf,'color','black'); %窗口背景黑色
    colordef black; %2D/3D图背景黑色
    plot3(x1, y1, z1, 'r.');
    grid on;
    hold on;
    plot3(x2, y2, z2, 'g.');
    hold off;
    axis equal
end

