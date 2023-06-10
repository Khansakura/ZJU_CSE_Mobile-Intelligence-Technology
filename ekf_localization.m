function [] = ekf_localization()
 
% Homework for ekf localization
% Modified by YH on 09/09/2019, thanks to the original open source
% Any questions please contact: zjuyinhuan@gmail.com

    close all;
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PxEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0.1 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
    %just choose the same parameter with the noise convariance matrix 
    convQ = diag([0.1 0.1 degreeToRadian(10)]).^2; 

    % Covariance Matrix for observation
    %just choose the same parameter with the noise convariance matrix 
    convR = diag([0.5 0.5 degreeToRadian(5)]).^2;
    % Other Intial
    % None

    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        xPred = doMotion(xEkf, u);
        %get the jacob A
        F = jacobF(xPred, u);
        %get the Prior error covariance
        PPred = F * PxEkf * F' + convQ;
        % Update 
        H=jacobH(xPred);
        %caculate the Kalman Gain
        K = PPred * H' * inv(H * PPred * H' + convR);
        %get the posterior x
        xEkf = xPred + K * (z - doObservation(z,xPred));
        %upgrade the error covariance
        PxEkf = (eye(size(K * H, 1)) - K * H) * PPred;
        % -----------------------------

        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 
end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    vx = u(1);
    vy = u(2);
    v = sqrt(vx^2+vy^2);
    M = [dt*cos(x(3)) 0 0;dt*sin(x(3)) 0 0;0 0 dt];
    x= x+M*[v,v,u(3)]';
end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    %get the linear velocity v
    vx = u(1);
    vy = u(2);
    v = sqrt(vx^2+vy^2);
    %caculate the Jacob matrix from dynamic equations ,here is the final
    %answer, the original equation is showed in the report paper.
    jF=[1 0 0;0 1 0;-dt*v*sin(x(1)) 1+dt*v*cos(x(1)) 1];
end

%Observation Model
function x = doObservation(z, xPred)
    %I just choose to get the mean value of the GPS and the X_pred, it
    %seems OK,but it is exactly not the best choice.
    x = 1/2.*(xPred+z);
 end

%Jacobian of Observation Model
function jH = jacobH(x)
    %the same as the doObservation, just use the I matrix to simplify the
    %Observation part.
    jH =[1 0 0;0 1 0;0 0 1];
end

% finally plot the results
function []=finalPlot(estimation)
    figure;
    
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
        
    % calculate error
    %define the error_sum
    error_xEkf=0;
    error_xOdom=0;
    %the length of the whole trajactory
    n = size(estimation.xTruth(:,1));
    %sum and save
    for i=1:n(1)
        x_real=estimation.xTruth(i,1);
        y_real=estimation.xTruth(i,2);
        xEkf=estimation.xEkf(i,1);
     	yEkf=estimation.xEkf(i,2);
        xOdom=estimation.xOdom(i,1);
        yOdom=estimation.xOdom(i,2);
        error_xEkf=error_xEkf+sqrt((x_real(1)-xEkf(1))^2+(y_real(1)-yEkf(1))^2);
        error_xOdom=error_xOdom+sqrt((x_real(1)-xOdom(1))^2+(y_real(1)-yOdom(1))^2);
    end
    %mean
    error_mean_xEKF = error_xEkf/n(1);
    error_mean_Odom = error_xOdom/n(1);
    %disp
    disp_xEKF = sprintf('EKF平均定位误差为：%f',error_mean_xEKF);
    disp(disp_xEKF);
    disp_Odom = sprintf('纯里程计平均定位误差为：%f',error_mean_Odom);
    disp(disp_Odom);
end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end