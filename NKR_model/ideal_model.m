close all
clear
lf = 0.4;
lr = 0.4;
lw = 0.3;
a = 0.05;
rw = 0.1;
dt = 0.01;
simTime = 20;
nSim = simTime / dt;
Animation_enable = 0;

wheel1.x = 0;
wheel1.y = 0;
wheel1.h = -pi/2;

UGV.lf = 0.4;
UGV.lr = 0.3;
UGV.lw = 0.3;
UGV.rw = 0.1;
UGV.frame = [lf,0;lf,lw/2;lf+a,0;lf,-lw/2;lf,0; 0,0;0,0.02;0,-0.02; 0,0;-lr,0;-lr,-lw/2;-lr+a,0;-lr,lw/2; -lr, 0];
Wheel.frame = [0.1,0.05; -0.1, 0.05; -0.1, -0.05; 0.1, -0.05; 0.1,0.05];
%% Arrays init
Time = zeros(nSim, 1);
V = zeros(nSim, 1);
Anr = zeros(nSim, 1);
Heading = zeros(nSim, 1);
X = zeros(nSim, 1);
Y = zeros(nSim, 1);
w = zeros(nSim, 4);
odo_w = zeros(nSim, 4);
gps_pos = zeros(nSim, 2);
gps_vel = zeros(nSim, 2);
gyro_anr = zeros(nSim, 1);
gamma = zeros(nSim, 6);
betta = zeros(nSim, 1);
odo_coords = zeros(nSim, 3);
wheels = zeros(nSim, 8);
% store navigation solution
sol = zeros(nSim, 9);
% sol key: [X Y Heading V dHeading w1 w2 w3 w4]

trajPhase = 1;
trajConst = 1.91;
trajTimer = 1.91;

odoX = 0;
odoY = 0;
odoHeading = -pi/2;

if Animation_enable == 1
    anim = figure('Name', 'Robot motion animation');
    axis equal
    grid on
end

%% Measurement errors parameters
odometer_error = 0.03; % percent (max error)
odometer_noise = 0.1; % rad/sec
gps_pos_error = 2.0; % meters (max error)
gps_pos_noise = 0.75; % meters
gps_vel_error = 0.2; % m/s (max error)
gps_vel_noise = 0.1; % m/s
gyro_bias = normrnd(0, 0.1); % rad/s
gyro_noise = 2*pi/180; % rad/s

% init robot struct
robot.x = 0;
robot.y = 0;
robot.heading = -pi/2;
robot.v = 0;
robot.dheading = 0;
robot.lf = 0.4;
robot.lr = 0.3;
robot.lw = 0.3;
robot.rw = 0.1;
robot.lsx = 0.15; % gnss antenna shift
robot.lsy = 0.05; % gnss antenna shift

%init Kalman state struct
kalman_state.X = zeros(7,1);
kalman_state.P = diag([10 10 3 5 5 5 5]);

%% Main cycle
for i = 1:nSim
    Time(i) = i * dt;
    V(i) = 5.236;
    %% Motion setup
    if Time(i) >= trajTimer
        trajPhase = trajPhase + 1;
        if rem(trajPhase, 2) == 0
            trajTimer = trajTimer + 9; % 9 seconds for rotation arc
        else
            trajTimer = trajTimer + 2*trajConst; % 3.82 seconds for straight line
        end
    end
    if rem(trajPhase, 2) ~= 0
        Anr(i) = 0;
        gamma(i,:) = [0, 0, 0, 0, 0, 0];
    else
        if rem(trajPhase, 4) == 2
            Anr(i) = pi/6; % turn right on every 2nd phase
            gamma(i,:) = [0, 0, 0, 0, 0, 0];
        else
            Anr(i) = -pi/6; % turn left on every 4th phase
            gamma(i,:) = [0, 0, 0, 0, 0, 0];
        end
    end
    %% Wheels kinematics
    % Linear speed of each wheel
    V1 = V(i); V2 = V(i); V3 = V(i); V4 = V(i);
    A1 = 0; A2 = 0; A3 = 0; A4 = 0;
    
    % If robot is turning - they differ
    if rem(trajPhase, 2) == 0
        V1 = abs(Anr(i)) * ( 10 + lw/2 * sign(Anr(i)) );
        V2 = abs(Anr(i)) * ( 10 - lw/2 * sign(Anr(i)) );
        V3 = abs(Anr(i)) * ( 10 + lw/2 * sign(Anr(i)) );
        V4 = abs(Anr(i)) * ( 10 - lw/2 * sign(Anr(i)) );
    end   
    
    betta(i) = 0; % assume that betta is quasi-constant
    % Also assume that gamma_f equals gamma_r, but has the opposite sign
    gamma(i,5) =  atan(Anr(i)*(lf + lr) / (V(i)*cos(betta(i))) / 2);
    gamma(i,6) = - gamma(i,5);
    dgammaf = gamma(i,5)^2 * lw / (2*lf + 2*lr);
    dgammar = gamma(i,6)^2 * lw / (2*lf + 2*lr);
    gamma(i,1) = gamma(i,5) + dgammaf*sign(gamma(i,5));
    gamma(i,2) = gamma(i,5) - dgammaf*sign(gamma(i,5));
    gamma(i,3) = gamma(i,6) + dgammar*sign(gamma(i,6));
    gamma(i,4) = gamma(i,6) - dgammar*sign(gamma(i,6));
    %% True model parameters
    if i == 1
        X(i) = 0;
        Y(i) = 0;
        Heading(i) = -pi/2;
    else
        Heading(i) = Heading(i-1) + Anr(i)*dt;
        X(i) = X(i-1) + V(i)*cos(Heading(i-1) + betta(i-1))*dt;
        Y(i) = Y(i-1) + V(i)*sin(Heading(i-1) + betta(i-1))*dt;
    end  
    %% True Wheel parameters
    wheels(i,1) = X(i) + lf*cos(Heading(i)) + lw/2*sin(Heading(i));
    wheels(i,2) = Y(i) + lf*sin(Heading(i)) - lw/2*cos(Heading(i));
    wheels(i,3) = X(i) + lf*cos(Heading(i)) - lw/2*sin(Heading(i));
    wheels(i,4) = Y(i) + lf*sin(Heading(i)) + lw/2*cos(Heading(i));
    wheels(i,5) = X(i) - lr*cos(Heading(i)) + lw/2*sin(Heading(i));
    wheels(i,6) = Y(i) - lr*sin(Heading(i)) - lw/2*cos(Heading(i));
    wheels(i,7) = X(i) - lr*cos(Heading(i)) - lw/2*sin(Heading(i));
    wheels(i,8) = Y(i) - lr*sin(Heading(i)) + lw/2*cos(Heading(i));
    if i ~= 1
        V1 = sqrt( (wheels(i,1) - wheels(i-1,1))^2 + (wheels(i,2) - wheels(i-1,2))^2 )/dt;
        V2 = sqrt( (wheels(i,3) - wheels(i-1,3))^2 + (wheels(i,4) - wheels(i-1,4))^2 )/dt;
        V3 = sqrt( (wheels(i,5) - wheels(i-1,5))^2 + (wheels(i,6) - wheels(i-1,6))^2 )/dt;
        V4 = sqrt( (wheels(i,7) - wheels(i-1,7))^2 + (wheels(i,8) - wheels(i-1,8))^2 )/dt;
    end
    w(i,:) = [V1/rw, V2/rw, V3/rw, V4/rw];
    aw1(i) = V(i)/rw;
    if rem(trajPhase, 2) == 0
        V1 = abs(Anr(i)) * sqrt(lf^2 + (10+sign(Anr(i))*lw/2)^2); % 
        aw1(i) = V1/rw;
    end
    %% Measurements model
    % Odometer errors
    odo_error1 = odometer_error * sin(Time(i)/20);
    odo_error2 = odometer_error * sin(Time(i)/16);
    odo_error3 = odometer_error * sin(Time(i)/12);
    odo_error4 = odometer_error * sin(Time(i)/8);
    odo_w(i,:) = [w(i,1) * (1 + odo_error1) + normrnd(0, odometer_noise),...
        w(i,2) * (1 + odo_error2) + normrnd(0, odometer_noise),...
        w(i,3) * (1 + odo_error3) + normrnd(0, odometer_noise),...
        w(i,4) * (1 + odo_error4) + normrnd(0, odometer_noise)];
    % GPS position errors
%     if rem(i,1/dt) == 0
    gps_pos_errorX = gps_pos_error * sin(Time(i)/25);
    gps_pos_errorY = gps_pos_error * cos(Time(i)/35);
    gps_pos(i,:) = [X(i) + gps_pos_errorX + normrnd(0, gps_pos_noise),...
        Y(i) + gps_pos_errorY + normrnd(0, gps_pos_noise)];
%     end
    % GPS velocity errors
%     if rem(i,1/dt) == 0
    gps_vel_errorX = gps_vel_error * cos(Time(i)/15);
    gps_vel_errorY = gps_vel_error * sin(Time(i)/8);
    gps_vel(i,:) = [V(i)*cos(Heading(i)) + gps_vel_errorX + normrnd(0, gps_vel_noise),...
        V(i)*sin(Heading(i)) + gps_vel_errorY + normrnd(0, gps_vel_noise)];
%     end
    % Gyroscope errors
    gyro_anr(i) = Anr(i) + gyro_bias + normrnd(0, gyro_noise);
    %% Baseline model
    odoV = rw/4 * (odo_w(i,1) + odo_w(i,2) + odo_w(i,3) + odo_w(i,4));
    odoBetta = atan( (lf*tan(gamma(i,6)) + lr*tan(gamma(i,5)))/(lf + lr) );
    odoAnr = odoV * cos(odoBetta) / (lr + lf) * (tan(gamma(i,5)) - tan(gamma(i,6)));
    odoHeading = odoHeading + odoAnr*dt;
    odoX = odoX + odoV*cos(odoHeading + odoBetta)*dt;
    odoY = odoY + odoV*sin(odoHeading + odoBetta)*dt;
    odo_coords(i,:) = [odoX, odoY, odoHeading];
    update.gps = 0;
    update.gyro = 0;
    if (rem(i,1) == 0)
        update.gyro = 1;
    end
    if (rem(i, 100) == 0)
        update.gps = 1;
    end
    % init sensors struct
    sensors.gps_error_pos = 2.0;
    sensors.gps_error_vel = 0.2;
    sensors.gyro_error = 2*pi/180;
    sensors.dt = dt;
    sensors.gps_x = gps_pos(i,1);
    sensors.gps_y = gps_pos(i,2);
    sensors.gps_dx = gps_vel(i,1);
    sensors.gps_dy = gps_vel(i,2);
    sensors.gyro = gyro_anr(i);
    % update robot struct
    robot.w1 = odo_w(i,1);
    robot.w2 = odo_w(i,2);
    robot.w3 = odo_w(i,3);
    robot.w4 = odo_w(i,4);
    robot.betta = odoBetta;
    robot.gammaf = gamma(i,6);
    robot.gammar = gamma(i,5);
    % call Kalman dilter
%     [robot, kalman_state] = navSys(robot, kalman_state, sensors, update);
%     sol(i,:) = [robot.x robot.y robot.heading robot.v robot.dheading robot.w1 robot.w2 robot.w3 robot.w4];
    % sol key: [X Y Heading V dHeading w1 w2 w3 w4]
    %% Animation
    if Animation_enable == 1
        trail = plot(Y(1:i),X(1:i), 'Color', [0.4 0.4 0.4], 'LineWidth', 0.5);
        UGVframe = rotateVectorImage2D(UGV.frame, Heading(i));
        frame = drawVectorImage2D(UGVframe, [Y(i) X(i)], anim, 'r');
        drawnow;
        delete(frame);
        delete(trail);

    end
end



an = figure('Name', 'Robot trajectory');
plot(Y,X, 'b', 'LineWidth', 1.0)
hold on
% plot(odo_coords(:,2),odo_coords(:,1), 'r', 'LineWidth', 0.5)
% plot(gps_pos(:,2),gps_pos(:,1), 'g', 'LineWidth', 0.5)
% plot(wheels(:,4), wheels(:,3), 'k', 'LineWidth', 0.25)
plot(wheels(:,2), wheels(:,1), 'g', 'LineWidth', 0.25)
% plot(wheels(:,6), wheels(:,5), 'c', 'LineWidth', 0.25)
% plot(wheels(:,8), wheels(:,7), 'y', 'LineWidth', 0.25)

axis equal
grid on

UGVframe = rotateVectorImage2D(UGV.frame, Heading(end));
frame1 = drawVectorImage2D(UGVframe, [Y(end) X(end)], an, 'r');
Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,1));
frame2 = drawVectorImage2D(Wheelframe, [wheels(end,2) wheels(end,1)], an, 'm');
Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,2));
frame3 = drawVectorImage2D(Wheelframe, [wheels(end,4) wheels(end,3)], an, 'm');
Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,3));
frame4 = drawVectorImage2D(Wheelframe, [wheels(end,6) wheels(end,5)], an, 'm');
Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,4));
frame5 = drawVectorImage2D(Wheelframe, [wheels(end,8) wheels(end,7)], an, 'm');

figure
plot(Time, w(:,1), 'b', 'LineWidth', 0.5)
hold on; grid on
plot(Time, aw1, 'r', 'LineWidth', 0.5)
% 
% figure;
% plot(Time, w, 'b')
% hold on
% grid on
% plot(Time, odo_w, 'r');
% 
% figure;
% plot(Time, Anr, 'b')
% hold on
% grid on
% plot(Time, gyro_anr, 'r');


% UGVframe = rotateVectorImage2D(UGV.frame, pi/4);
% graph = figure;
% axis equal
% drawVectorImage2D(UGVframe, [1 10], graph, 'r');

% figure;
% plot([Time(1) Time(end)], [V V], 'b')
% hold on
% grid on
% plot(Time, asssb, 'r');

% an = figure('Name', 'Robot trajectory');
% plot(odo_coords(:,2), odo_coords(:,1), 'b', 'LineWidth', 1.0)
% grid on
% axis equal
% hold on
% plot(gps_pos(:,2), gps_pos(:,1), 'r', 'LineWidth', 1.0)
% plot(sol(:,2), sol(:,1), 'k', 'LineWidth', 1.0)
% 
% figure;
% plot(Time, sol(6), 'r');
% hold on
% grid on
% plot(Time, odo_w(i,1));
% 
