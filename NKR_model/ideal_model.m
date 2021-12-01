close all
clear
lf = 0.4;
lr = 0.4;
lw = 0.6;
a = 0.05; % nkr wheel scaling for drawing purpose
rw = 0.15;
dt = 0.01;
simTime = 20;
nSim = simTime / dt;

wheel1.x = 0;
wheel1.y = 0;
wheel1.h = -pi/2;

UGV.frame = [lf,0;lf,lw/2;lf+a,0;lf,-lw/2;lf,0; 0,0;0,0.02;0,-0.02; 0,0;-lr,0;-lr,-lw/2;-lr+a,0;-lr,lw/2; -lr, 0];
Wheel.frame = [0.1,0.05; -0.1, 0.05; -0.1, -0.05; 0.1, -0.05; 0.1,0.05];
%% Arrays init
Time = zeros(nSim, 1);
% NKR params
V = zeros(nSim, 1);
Anr = zeros(nSim, 1);
Heading = zeros(nSim, 1);
X = zeros(nSim, 1);
Y = zeros(nSim, 1);
w = zeros(nSim, 4);
gamma = zeros(nSim, 2);
betta = zeros(nSim, 5);
% Measurements
odo_w = zeros(nSim, 4);
gps_pos = zeros(nSim, 2);
gps_vel = zeros(nSim, 2);
gyro_anr = zeros(nSim, 1);

odo_coords = zeros(nSim, 3);
wheels = zeros(nSim, 8);
% store navigation solution
% sol = zeros(nSim, 9);
% sol key: [X Y Heading V dHeading w1 w2 w3 w4]
V1 = zeros(nSim, 1);
V2 = zeros(nSim, 1);
V3 = zeros(nSim, 1);
V4 = zeros(nSim, 1);
V11 = zeros(nSim, 1);
V22 = zeros(nSim, 1);
V33 = zeros(nSim, 1);
V44 = zeros(nSim, 1);

trajPhase = 1;
trajConst = 1.91;
trajTimer = 1.91;

odoX = 0;
odoY = 0;
odoHeading = 0;


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
% robot.x = 0;
% robot.y = 0;
% robot.heading = 0;
% robot.v = 0;
% robot.dheading = 0;
% robot.lf = 0.4;
% robot.lr = 0.3;
% robot.lw = 0.3;
% robot.rw = 0.1;
% robot.lsx = 0.15; % gnss antenna shift
% robot.lsy = 0.05; % gnss antenna shift

%init Kalman state struct
% kalman_state.X = zeros(7,1);
% kalman_state.P = diag([10 10 3 5 5 5 5]);

%% Main cycle
for i = 1:nSim
    Time(i) = i * dt;
    V(i) = 5.236; % constant velocity
    %% Motion setup
    if Time(i) >= trajTimer
        trajPhase = trajPhase + 1;
        if rem(trajPhase, 2) == 0
            trajTimer = trajTimer + 9; % 9 seconds for rotation arc
        else
            trajTimer = trajTimer + 2*trajConst; % 3.82 seconds for straight line
        end
        if rem(trajPhase, 5) == 0
%             V(i) = V(i) - 1;
        end
    end
    gamma_mean = 0;
    if rem(trajPhase, 2) ~= 0 % odd phase
        Anr(i) = 0;
        gamma(i,:) = [0, 0];
    else                        % even phase
        if rem(trajPhase, 4) == 2
%             Anr(i) = pi/6; % turn right on every 2nd phase
            gamma_mean = 0.0799;% will be equal to 0.5236 rad/s
            gamma(i,1) = gamma_mean - gamma_mean^2*lw/(lf+lr);
            gamma(i,2) = gamma_mean + gamma_mean^2*lw/(lf+lr);
        else
%             Anr(i) = -pi/6; % turn left on every 4th phase
            gamma_mean = -0.0799; % will be equal to -0.5236 rad/s
            gamma(i,1) = gamma_mean - gamma_mean^2*lw/(lf+lr);
            gamma(i,2) = gamma_mean + gamma_mean^2*lw/(lf+lr);
        end
    end
    %% Wheels kinematics
    % Linear speed of each wheel
    V1(i) = V(i); V2(i) = V(i); V3(i) = V(i); V4(i) = V(i);
    A1 = 0; A2 = 0; A3 = 0; A4 = 0;
       
    Vxb = 0.25 * ( V(i)*cos(gamma(i,1)) + V(i)*cos(gamma(i,2)) + V(i) + V(i));
    Vyb = 0.25 * ( V(i)*sin(gamma(i,1)) + V(i)*sin(gamma(i,2)));
    betta(i) = atan(Vyb/Vxb);
    Anr(i) = V(i)*cos(betta(i))*tan(gamma_mean) / (lf + lr);
    Rc = abs(V(i) / Anr(i));
    
    % If robot is turning - they differ
    if rem(trajPhase, 2) == 0
        V1(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc + lw/2 * sign(Anr(i)))^2);
        V2(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc - lw/2 * sign(Anr(i)))^2);
        V3(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc + lw/2 * sign(Anr(i)))^2);
        V4(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc - lw/2 * sign(Anr(i)))^2);
    end  

    %% True model parameters
    if i == 1
        X(i) = 0;
        Y(i) = 0;
        Heading(i) = 0;
    else
        Heading(i) = Heading(i-1) + Anr(i)*dt;
        X(i) = X(i-1) + V(i)*cos(Heading(i) + betta(i))*dt;
        Y(i) = Y(i-1) + V(i)*sin(Heading(i) + betta(i))*dt;
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
%     odoV = rw/4 * (odo_w(i,1) + odo_w(i,2) + odo_w(i,3) + odo_w(i,4));
%     odoBetta = atan( (lf*tan(gamma(i,6)) + lr*tan(gamma(i,5)))/(lf + lr) );
%     odoAnr = odoV * cos(odoBetta) / (lr + lf) * (tan(gamma(i,5)) - tan(gamma(i,6)));
%     odoHeading = odoHeading + odoAnr*dt;
%     odoX = odoX + odoV*cos(odoHeading + odoBetta)*dt;
%     odoY = odoY + odoV*sin(odoHeading + odoBetta)*dt;
%     odo_coords(i,:) = [odoX, odoY, odoHeading];
%     update.gps = 0;
%     update.gyro = 0;
%     if (rem(i,1) == 0)
%         update.gyro = 1;
%     end
%     if (rem(i, 100) == 0)
%         update.gps = 1;
%     end
%     % init sensors struct
%     sensors.gps_error_pos = 2.0;
%     sensors.gps_error_vel = 0.2;
%     sensors.gyro_error = 2*pi/180;
%     sensors.dt = dt;
%     sensors.gps_x = gps_pos(i,1);
%     sensors.gps_y = gps_pos(i,2);
%     sensors.gps_dx = gps_vel(i,1);
%     sensors.gps_dy = gps_vel(i,2);
%     sensors.gyro = gyro_anr(i);
%     % update robot struct
%     robot.w1 = odo_w(i,1);
%     robot.w2 = odo_w(i,2);
%     robot.w3 = odo_w(i,3);
%     robot.w4 = odo_w(i,4);
%     robot.betta = odoBetta;
%     robot.gammaf = gamma(i,6);
%     robot.gammar = gamma(i,5);
    % call Kalman dilter
%     [robot, kalman_state] = navSys(robot, kalman_state, sensors, update);
%     sol(i,:) = [robot.x robot.y robot.heading robot.v robot.dheading robot.w1 robot.w2 robot.w3 robot.w4];
    % sol key: [X Y Heading V dHeading w1 w2 w3 w4]
    %% Animation

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

% UGVframe = rotateVectorImage2D(UGV.frame, Heading(end));
% frame1 = drawVectorImage2D(UGVframe, [Y(end) X(end)], an, 'r');
% Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,1));
% frame2 = drawVectorImage2D(Wheelframe, [wheels(end,2) wheels(end,1)], an, 'm');
% Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,2));
% frame3 = drawVectorImage2D(Wheelframe, [wheels(end,4) wheels(end,3)], an, 'm');
% Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,3));
% frame4 = drawVectorImage2D(Wheelframe, [wheels(end,6) wheels(end,5)], an, 'm');
% Wheelframe = rotateVectorImage2D(Wheel.frame, Heading(end) + gamma(end,4));
% frame5 = drawVectorImage2D(Wheelframe, [wheels(end,8) wheels(end,7)], an, 'm');

figure
plot(Time, V1, 'b', 'LineWidth', 0.5)
hold on; grid on
plot(Time, V2, 'r', 'LineWidth', 0.5)
plot(Time, V11, '--b', 'LineWidth', 0.5)
plot(Time, V22, '--r', 'LineWidth', 0.5)
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
% V11(500)/Anr(500)
% V22(500)/Anr(500)
% V11(500)/Anr(500) - V22(500)/Anr(500)
% wf = atan(lw / 2/lf);
% Lf = sqrt(lf^2 + 0.25*lw^2);
% dR1 = sin(wf) * Lf / sin(pi - wf - (pi/2 - wf - gamma(500,1)))
% dR1 = lw/2 / cos(gamma(500,1))
% gip = sqrt(lf^2 + (10+lw/2)^2)
% gip2= sqrt(lf^2 + (10-lw/2)^2)
% V01 = Anr(500) * gip
% V02 = Anr(500) * gip2
% maybe = sin(pi/2+wf)/sin(pi/2-wf-gamma(500,1))*10