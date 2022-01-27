close all
clear
lf = 0.4;
lr = 0.4;
lw = 0.3;
a = 0.05; % arrow len (display only)
rw = 0.1;
dt = 0.01;
simTime = 200;
nSim = simTime / dt;
Animation_enable = 0;

wheel1.x = 0;
wheel1.y = 0;
wheel1.h = -pi/2;

referenceLat = 55.367812;
referenceLon = 37.108906;
referenceTime = 100000;

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
Errors = zeros(nSim, 5);
TELEMETRY = zeros(nSim, 12);
%[ timestamp, gamma1, gamma2, gyro, lat, lon, Vnorth, Veast, w1, w2, w3, w4]
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
odometer_error = 0.01; % percent (max error)
odometer_noise = 0.15; % rad/sec
gps_pos_error = 2.0; % meters (max error)
gps_pos_noise = 0.75; % meters
gps_vel_error = 0.2; % m/s (max error)
gps_vel_noise = 0.1; % m/s
gyro_bias = normrnd(0, 0.1); % rad/s
gyro_noise = 2*pi/180; % rad/s


%init Kalman state struct
kalman_state.X = zeros(9,1);
kalman_state.X(3) = 0;%-pi/2; % set initial heading
kalman_state.P = diag([100 100 10 10 10 100 100 10 10]);

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
    
    betta(i) = 0;%Anr(i)/25; % assume that betta is quasi-constant
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
    testa(i) = (V1 + V3 - V2 - V4)/2 / lw;
    %% Measurements model
    % Odometer errors
    odo_error1 = odometer_error * sin(Time(i)/50);
    odo_error2 = odometer_error * cos(Time(i)/36);
    odo_error3 = odometer_error * cos(Time(i)/24);
    odo_error4 = odometer_error * sin(Time(i)/18);
    odo_w(i,:) = [w(i,1) * (1 + odo_error1) + normrnd(0, odometer_noise),...
        w(i,2) * (1 + odo_error2) + normrnd(0, odometer_noise),...
        w(i,3) * (1 + odo_error3) + normrnd(0, odometer_noise),...
        w(i,4) * (1 + odo_error4) + normrnd(0, odometer_noise)];
    % GPS position errors
    lsx = 0.15;
    lsy = 0.05;
%     if rem(i,1/dt) == 0
    gps_pos_errorX = gps_pos_error * sin(Time(i)/25) + lsx*cos(Heading(i)) - lsy*sin(Heading(i));
    gps_pos_errorY = gps_pos_error * cos(Time(i)/35) + lsx*sin(Heading(i)) + lsy*cos(Heading(i));
    gps_pos(i,:) = [X(i) + gps_pos_errorX + normrnd(0, gps_pos_noise),...
        Y(i) + gps_pos_errorY + normrnd(0, gps_pos_noise)];
%     end
    % GPS velocity errors
%     if rem(i,1/dt) == 0
    gps_vel_errorX = gps_vel_error * cos(Time(i)/15) + Anr(i)*(lsx*cos(Heading(i)) - lsy*sin(Heading(i)));
    gps_vel_errorY = gps_vel_error * sin(Time(i)/8) + Anr(i)*(lsx*sin(Heading(i)) + lsy*cos(Heading(i)));
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
    kalman_state.pos = 2e-5;
    kalman_state.h = 1e-4;
    kalman_state.v = 3e-3;
    kalman_state.dh = 8e-4;
    kalman_state.odo = 1e-2;%8e-3;
%     kalman_state.U = [2e-5 2e-5 1e-5 1e-2 1e-3 8e-9 8e-9 8e-9 8e-9]';
    % init sensors struct
    sensors.gps_error_pos = 3e0;
    sensors.gps_error_vel = 6e-1;
    sensors.gyro_error = 4e-1;
    sensors.odo_error = 1e0;%8e-1;
    
    
    sensors.dt = dt;
    sensors.gps_x = gps_pos(i,1);
    sensors.gps_y = gps_pos(i,2);
    sensors.gps_dx = gps_vel(i,1);
    sensors.gps_dy = gps_vel(i,2);
    sensors.gyro = gyro_anr(i);
    sensors.w1 = odo_w(i,1);
    sensors.w2 = odo_w(i,2);
    sensors.w3 = odo_w(i,3);
    sensors.w4 = odo_w(i,4);
    sensors.gammaf = gamma(i,5);
    sensors.gammar = gamma(i,6);
    
    % call Kalman dilter
%     kalman_state = forwardNav(kalman_state, sensors, update);
    kalman_state = ukfNav(kalman_state, sensors, update);
    
    sol(i,:) = kalman_state.X;
%     % sol key: [X Y Heading V dHeading w1 w2 w3 w4]

    Errors(i,1) = sqrt( (sol(i,1) - X(i))^2 + (sol(i,2) - Y(i))^2 );
    Errors(i,2) = sqrt( (odo_coords(i,1) - X(i))^2 + (odo_coords(i,2) - Y(i))^2 );
    Errors(i,3) = sqrt( (gps_pos(i,1) - X(i))^2 + (gps_pos(i,2) - Y(i))^2 );
    Errors(i,4) = sqrt(sqrt( kalman_state.P(1,1)^2 +kalman_state.P(2,2)^2 ));
    
    %% Build telemetry file
    % Timestamp
    TELEMETRY(i, 1) = 10 * i + referenceTime; % ms
    % Gamma1 and Gamma2
    TELEMETRY(i, 2) = gamma(i,5); % rad
    TELEMETRY(i, 3) = gamma(i,6); % rad
    % Gyro
    TELEMETRY(i, 4) = sensors.gyro;
    % Lat and Lon
    TELEMETRY(i, 5) = referenceLat + sensors.gps_x / 111111;
    TELEMETRY(i, 6) = referenceLon + sensors.gps_y / (111111 * cos(referenceLat));
    % North and East velocity
    TELEMETRY(i, 7) = sensors.gps_dx;
    TELEMETRY(i, 8) = sensors.gps_dy;
    % Odometry
    TELEMETRY(i, 9) = sensors.w1;
    TELEMETRY(i, 10) = sensors.w2;
    TELEMETRY(i, 11) = sensors.w3;
    TELEMETRY(i, 12) = sensors.w4;
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
plot(gps_pos(:,2), gps_pos(:,1), '.r', 'MarkerSize', 0.25)
hold on
% plot(odo_coords(:,2),odo_coords(:,1), 'r', 'LineWidth', 0.5)
% plot(gps_pos(:,2),gps_pos(:,1), 'g', 'LineWidth', 0.5)
% plot(wheels(:,4), wheels(:,3), 'b', 'LineWidth', 0.25)
% plot(wheels(:,2), wheels(:,1), 'g', 'LineWidth', 0.25)
% plot(wheels(:,6), wheels(:,5), 'c', 'LineWidth', 0.25)
% plot(wheels(:,8), wheels(:,7), 'y', 'LineWidth', 0.25)
plot(odo_coords(:,2), odo_coords(:,1), 'b', 'LineWidth', 0.5)
plot(sol(:,2), sol(:,1), 'g', 'LineWidth', 1.5)
plot(Y,X, 'k', 'LineWidth', 1.5)

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



figure;
plot(Time, X, 'b')
grid on
hold on
plot(Time, Y, 'r');
plot(Time, gps_pos(:,1), '.b', 'MarkerSize', 0.3);
plot(Time, gps_pos(:,2), '.r', 'MarkerSize', 0.3);
plot(Time, sol(:,1), '--b', 'Color', [0 0 0.6], 'LineWidth', 1.5);
plot(Time, sol(:,2), '--r', 'Color', [0.6 0 0], 'LineWidth', 1.5);

title('Coords')

figure;
plot(Time, Heading, 'b')
grid on
hold on
plot(Time, sol(:,3), 'r');
title('Heading')

figure;
plot(Time, V, 'b')
grid on
hold on
plot(Time, Anr, 'r');
plot(Time, gyro_anr, '.r', 'MarkerSize', 0.3);
plot(Time, sol(:,4), '--b');
plot(Time, sol(:,5), '--r', 'Color', [0.6 0 0], 'LineWidth', 1.5);
title('Velocities')

figure;
plot(Time, w(:,1), 'b')
grid on
hold on
plot(Time, w(:,2), 'r');
plot(Time, odo_w(:,1), '.b', 'MarkerSize', 0.4);
plot(Time, odo_w(:,2), '.r', 'MarkerSize', 0.4);
plot(Time, sol(:,6), '--b', 'Color', [0 0 0.6], 'LineWidth', 1.5);
plot(Time, sol(:,7), '--r', 'Color', [0.6 0 0], 'LineWidth', 1.5);
title('Front wheels')

figure;
plot(Time, w(:,3), 'b')
grid on
hold on
plot(Time, w(:,4), 'r');
plot(Time, odo_w(:,3), '.b', 'MarkerSize', 0.4);
plot(Time, odo_w(:,4), '.r', 'MarkerSize', 0.4);
plot(Time, sol(:,8), '--b', 'Color', [0 0 0.6], 'LineWidth', 1.5);
plot(Time, sol(:,9), '--r', 'Color', [0.6 0 0], 'LineWidth', 1.5);
title('Rear wheels')


figure
plot(Time, Errors(:,3), '.r', 'MarkerSize', 0.1);
grid on
hold on
plot(Time, Errors(:,2), 'b', 'LineWidth', 2);
plot(Time, Errors(:,1), 'g', 'LineWidth', 3);
shift = std(Errors(:,1));
plot([Time(1) Time(end)], [mean(Errors(:,1)) mean(Errors(:,1))], 'k', 'LineWidth', 1.5);
plot([Time(1) Time(end)], [mean(Errors(:,2)) mean(Errors(:,2))], 'b', 'LineWidth', 1.0);
plot([Time(1) Time(end)], [mean(Errors(:,3)) mean(Errors(:,3))], 'r', 'LineWidth', 1.0);
plot([Time(1) Time(end)], [mean(Errors(:,1))+shift mean(Errors(:,1))+shift], '--k', 'LineWidth', 1.5);
plot([Time(1) Time(end)], [mean(Errors(:,1))-shift mean(Errors(:,1))-shift], '--k', 'LineWidth', 1.5);
plot(Time, Errors(:,1) + Errors(:,4), '--g', 'LineWidth', 1);
plot(Time, Errors(:,1) - Errors(:,4), '--g', 'LineWidth', 1);
fprintf('error mean: %.2f std: %.2f\n', mean(Errors(:,1)), shift)

figure;
plot(Time, Anr, 'b')
hold on
grid on
plot(Time, testa, 'r')