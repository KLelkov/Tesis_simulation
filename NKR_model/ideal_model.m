close all
clear
lf = 0.4;
lr = 0.4;
lw = 0.6;
a = 0.05; % nkr wheel scaling for drawing purpose
rw = 0.254/2;
dt = 0.01;
simTime = 55;
nSim = simTime / dt;


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
Betta = zeros(nSim, 4);
% Measurements
odo_w = zeros(nSim, 4);
gps_pos = zeros(nSim, 2);
gps_vel = zeros(nSim, 2);
gyro_anr = zeros(nSim, 1);
odo_gamma = zeros(nSim, 1);
Kappa = zeros(nSim, 1);

odo_coords = zeros(nSim, 3);
% store navigation solution

V1 = zeros(nSim, 1);
V2 = zeros(nSim, 1);
V3 = zeros(nSim, 1);
V4 = zeros(nSim, 1);

trajPhase = 1;
trajConst = 1.91;
trajTimer = 1.91;

odoX = 0;
odoY = 0;
odoHeading = 0;


%% Measurement errors parameters
odometer_error = 0.00; % percent (max error)
odometer_noise = 0.0; % rad/sec
gps_pos_error = 2.0; % meters (max error)
gps_pos_noise = 0.75; % meters
gps_vel_error = 0.2; % m/s (max error)
gps_vel_noise = 0.1; % m/s
gyro_bias = 0;%normrnd(0, 0.1); % rad/s
gyro_noise = 2*pi/180; % rad/s
odo_gamma_noise = 0.010; % rad

%% Slipping parameters
slipping_period1 = 30;
slipping_period2 = 55;
slipping_amp1 = 0;%0.02;
slipping_amp2 = 0;%-0.03;


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
    end
    gamma_mean = 0;
    
    % slipping angles
    Betta(i,1) = slipping_amp1 * sin(Time(i)/slipping_period1);
    Betta(i,2) = slipping_amp2 * cos(Time(i)/slipping_period2);
    Betta(i,3) = slipping_amp2 * sin(Time(i)/slipping_period1);
    Betta(i,4) = slipping_amp1 * sin(Time(i)/slipping_period2);
    
    if rem(trajPhase, 2) ~= 0 % odd phase
        Anr(i) = 0;
        gamma(i,:) = [0, 0];
    else                        % even phase
        if rem(trajPhase, 4) == 2
%             Anr(i) = pi/6; % turn right on every 2nd phase
            % gamma_mean + (betta1 + betta2)/2 = atan(tanval)
            % tanval - tan((betta3 + betta4)/2) = tan(0.0799)
            % tanval = tan(gamma_mean + (betta1 + betta2)/2)
            tanval =  tan(0.0799) + tan((Betta(i,3) + Betta(i,4)) / 2);
            gamma_mean = atan(tanval) - (Betta(i,1) + Betta(i,2)) / 2;
%             gamma_mean = 0.0799 ;% will be equal to 0.5236 rad/s
            gamma(i,1) = gamma_mean - gamma_mean^2*lw/(lf+lr);
            gamma(i,2) = gamma_mean + gamma_mean^2*lw/(lf+lr);
        else
%             Anr(i) = -pi/6; % turn left on every 4th phase
            %gamma_mean = -0.0799; % will be equal to -0.5236 rad/s
            tanval =  tan(-0.0799) + tan((Betta(i,3) + Betta(i,4)) / 2);
            gamma_mean = atan(tanval) - (Betta(i,1) + Betta(i,2)) / 2;
            gamma(i,1) = gamma_mean - gamma_mean^2*lw/(lf+lr);
            gamma(i,2) = gamma_mean + gamma_mean^2*lw/(lf+lr);
        end
    end
    %% Wheels kinematics
    % Linear speed of each wheel
    V1(i) = V(i); V2(i) = V(i); V3(i) = V(i); V4(i) = V(i);
    A1 = 0; A2 = 0; A3 = 0; A4 = 0;
    

    Vxb = 0.25 * ( V(i)*cos(gamma(i,1) + Betta(i,1)) + V(i)*cos(gamma(i,2) + Betta(i,2)) + V(i)*cos(Betta(i,3)) + V(i)*cos(Betta(i,4)));
    Vyb = 0.25 * ( V(i)*sin(gamma(i,1) + Betta(i,1)) + V(i)*sin(gamma(i,2) + Betta(i,2)) + V(i)*sin(Betta(i,3)) + V(i)*sin(Betta(i,4)));
    B = atan(Vyb/Vxb);
    Anr(i) = V(i)*cos(B)*(tan(gamma_mean + 0.5*Betta(i,1) + 0.5*Betta(i,2)) - tan(0.5*Betta(i,3) + 0.5*Betta(i,4))) / (lf + lr);
    Rc = abs(V(i) / Anr(i));
    Kappa(i) = B;
    
    % If robot is turning - they differ
    if rem(trajPhase, 2) == 0
        V1(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc + lw/2 * sign(Anr(i)))^2);
        V2(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc - lw/2 * sign(Anr(i)))^2);
        V3(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc + lw/2 * sign(Anr(i)))^2);
        V4(i) = abs(Anr(i)) * sqrt(lf^2 + ( Rc - lw/2 * sign(Anr(i)))^2);
    end
    w(i,:) = [V1(i)/rw V2(i)/rw V3(i)/rw V4(i)/rw];

    %% True model parameters
    if i == 1
        X(i) = 0;
        Y(i) = 0;
        Heading(i) = 0;
    else
        Heading(i) = Heading(i-1) + Anr(i)*dt;
        X(i) = X(i-1) + V(i)*cos(Heading(i) + B)*dt;
        Y(i) = Y(i-1) + V(i)*sin(Heading(i) + B)*dt;
    end  

    %% Measurements model
    % Odometer errors
    odo_error1 = odometer_error * sin(Time(i)/25);
    odo_error2 = odometer_error * sin(Time(i)/18);
    odo_error3 = odometer_error * sin(Time(i)/11);
    odo_error4 = odometer_error * sin(Time(i)/8);
    odo_w(i,:) = [w(i,1) * (1 + odo_error1) + normrnd(0, odometer_noise),...
        w(i,2) * (1 + odo_error2) + normrnd(0, odometer_noise),...
        w(i,3) * (1 + odo_error3) + normrnd(0, odometer_noise),...
        w(i,4) * (1 + odo_error4) + normrnd(0, odometer_noise)];
    odo_gamma(i) = gamma_mean + normrnd(0, odo_gamma_noise);
%     odo_gamma(i) = gamma_error - gamma_error^2*lw/(lf+lr);
%     odo_gamma(i,2) = gamma_error + gamma_error^2*lw/(lf+lr);
    % GPS position errors
    if rem(i, 50) == 0 || i == 1
        gps_pos_errorX = gps_pos_error * sin(Time(i)/25);
        gps_pos_errorY = gps_pos_error * cos(Time(i)/35);
        gps_pos(i,:) = [X(i) + gps_pos_errorX + normrnd(0, gps_pos_noise),...
        Y(i) + gps_pos_errorY + normrnd(0, gps_pos_noise)];
        % GPS velocity errors
        gps_vel_errorX = gps_vel_error * cos(Time(i)/15);
        gps_vel_errorY = gps_vel_error * sin(Time(i)/8);
        gps_vel(i,:) = [V(i)*cos(Heading(i)) + gps_vel_errorX + normrnd(0, gps_vel_noise),...
        V(i)*sin(Heading(i)) + gps_vel_errorY + normrnd(0, gps_vel_noise)];
    else
        gps_pos(i,:) = gps_pos(i-1,:);
        gps_vel(i,:) = gps_vel(i-1,:);
    end
    % Gyroscope errors
    gyro_anr(i) = Anr(i) + gyro_bias + normrnd(0, gyro_noise);
    %% Baseline model

end



figure('Name', 'Robot trajectory');
plot(Y,X, 'b', 'LineWidth', 1.0)
axis equal
grid on
hold on
plot(gps_pos(:,2), gps_pos(:,1), 'r', 'LineWidth', 1.0)

figure('Name', 'Robot rotation velocity');
plot(Time, Anr, 'b', 'LineWidth', 1.0)
grid on
hold on
plot(Time, gyro_anr, 'r', 'LineWidth', 1.0)
legend Anr Gyro

figure('Name', 'Robot velocity');
plot(Time, V, 'b', 'LineWidth', 1.0)
grid on
hold on
plot(Time, sqrt(gps_vel(:,1).^2 + gps_vel(:,2).^2), 'r', 'LineWidth', 1.0)
legend V gpsVel

figure('Name', 'Encoder readings');
plot(Time, w(:,1), 'b', 'LineWidth', 1.0)
grid on
hold on
plot(Time, w(:,2), '--b', 'LineWidth', 1.0)
plot(Time, odo_w(:,1), 'r', 'LineWidth', 1.0)
legend omega1 omega2 odo1

figure('Name', 'Gamma readings');
plot(Time, gamma(:,1), 'b', 'LineWidth', 1.0)
grid on
hold on
plot(Time, gamma(:,2), 'r', 'LineWidth', 1.0)
plot(Time, odo_gamma, 'g', 'LineWidth', 1.0)
% plot(Time, odo_gamma(:,2), '--r', 'LineWidth', 1.0)
legend gamma1 gamma2 odo

clearvars -except Kappa odo_gamma Time odo_w gps_vel gps_pos gyro_anr X Y Heading Betta V Anr w gamma

