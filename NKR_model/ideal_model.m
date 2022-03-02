close all
clear
lf = 0.4;
lr = 0.4;
lw = 0.6;
a = 0.05; % nkr wheel scaling for drawing purpose
rw = 0.254/2;
dt = 0.01;
simTime = 900;
nSim = simTime / dt;

lsx = -0.5;
lsy = 0.1;


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

gps_pos_sci = zeros(nSim, 2);
gps_delta = zeros(nSim, 2);
gps_ddelta = zeros(nSim, 2);
% store navigation solution

V1 = zeros(nSim, 1);
V2 = zeros(nSim, 1);
V3 = zeros(nSim, 1);
V4 = zeros(nSim, 1);

trajPhase = 1;
trajConst = 1.91;
trajTimer = 33.9539;

odoX = 0;
odoY = 0;
odoHeading = 0;


%% Measurement errors parameters
odometer_error = 0.0; % percent (max error)
odometer_noise = 0.1; % rad/sec
gps_pos_error = 2.0; % meters (max error)
gps_pos_noise = 0.75; % meters
gps_vel_error = 0.4; % m/s (max error)
gps_vel_noise = 0.1; % m/s
gyro_bias = 0;%normrnd(0, 0.1); % rad/s
gyro_noise = 3*pi/180; % rad/s
odo_gamma_noise = 0.010; % rad

%% Slipping parameters
slipping_period1 = 70;
slipping_period2 = 185;
slipping_amp1 = 0.017;
slipping_amp2 = -0.025;


r = 30;
fprintf("Trajectory len: %.1f\n", (1.5*pi*r + 2*r)*2);


%% Main cycle
for i = 1:nSim
    Time(i) = i * dt;
    V(i) = 1.7671; % constant velocity
    %% Motion setup
    if Time(i) >= trajTimer
        trajPhase = trajPhase + 1;
        if rem(trajPhase, 2) == 0
            trajTimer = trajTimer + 80 ; % 9 seconds for rotation arc
            
        else
            trajTimer = trajTimer + 33.9539*2; % 16.9765 seconds for straight line
            
        end
    end
    gamma_mean = 0;
    
    % slipping angles
    Betta(i,1) = -slipping_amp1 * sin(Time(i)/slipping_period1);
    Betta(i,2) = slipping_amp2 * cos(Time(i)/slipping_period2);
    Betta(i,3) = -slipping_amp2 * sin(Time(i)/slipping_period1);
    Betta(i,4) = slipping_amp1 * sin(Time(i)/slipping_period2);
%     
    if rem(trajPhase, 2) ~= 0 % odd phase
        Anr(i) = 0;
        gamma(i,:) = [0, 0];
        V(i) = 1.7671/2;
        tanval = 0;
        tanf = tanval + tan((Betta(i,3) + Betta(i,4)) / 2);
        gamma_mean = atan(tanf) - (Betta(i,1) + Betta(i,2)) / 2;
        gamma(i,1) = gamma_mean - gamma_mean^2*lw/(lf+lr);
        gamma(i,2) = gamma_mean + gamma_mean^2*lw/(lf+lr);
    else                        % even phase
        V(i) = 1.7671;
        if rem(trajPhase, 4) == 2
            tanval = 0.0267; % if overall tangent equals to this value,
                             % the robot will move right (V=1.7671, Anr = 0.0589)
            % Lets find the gamma value the robot needs to compensate for
            % the wheels slip
            tanf = tanval + tan((Betta(i,3) + Betta(i,4)) / 2);
            gamma_mean = atan(tanf) - (Betta(i,1) + Betta(i,2)) / 2;
            gamma(i,1) = gamma_mean - gamma_mean^2*lw/(lf+lr);
            gamma(i,2) = gamma_mean + gamma_mean^2*lw/(lf+lr);
        else
            tanval = -0.0267;
            tanf = tanval + tan((Betta(i,3) + Betta(i,4)) / 2);
            gamma_mean = atan(tanf) - (Betta(i,1) + Betta(i,2)) / 2;
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
        V3(i) = abs(Anr(i)) * sqrt(lr^2 + ( Rc + lw/2 * sign(Anr(i)))^2);
        V4(i) = abs(Anr(i)) * sqrt(lr^2 + ( Rc - lw/2 * sign(Anr(i)))^2);
    end
    w(i,:) = [V1(i)/rw V2(i)/rw V3(i)/rw V4(i)/rw];

    %% True model parameters
    if i == 1
        X(i) = 0;
        Y(i) = 0;
        Heading(i) = 0;
    else
        Heading(i) = wrapToPi(Heading(i-1) + Anr(i)*dt);
        X(i) = X(i-1) + V(i)*cos(Heading(i) + B)*dt;
        Y(i) = Y(i-1) + V(i)*sin(Heading(i) + B)*dt;
    end  

    %% Measurements model
    % Odometer errors
    odo_error1 = odometer_error * sin(Time(i)/85);
    odo_error2 = odometer_error * sin(Time(i)/49);
    odo_error3 = odometer_error * sin(Time(i)/30);
    odo_error4 = odometer_error * sin(Time(i)/28);
    odo_w(i,:) = [w(i,1) * (1 + odo_error1) + normrnd(0, odometer_noise),...
        w(i,2) * (1 + odo_error2) + normrnd(0, odometer_noise),...
        w(i,3) * (1 + odo_error3) + normrnd(0, odometer_noise),...
        w(i,4) * (1 + odo_error4) + normrnd(0, odometer_noise)];
    odo_gamma(i) = gamma_mean + normrnd(0, odo_gamma_noise);
%     odo_gamma(i) = gamma_error - gamma_error^2*lw/(lf+lr);
%     odo_gamma(i,2) = gamma_error + gamma_error^2*lw/(lf+lr);
    % GPS position errors
    gps_angle = pi/6; % ?
    mu = 0.1;
    delta = 2.2;
    if (Time(i) >= 600)
        delta = -1.4;
    end
    sigma_w = 2.1;
    sigma = 1.8;

    if i > 1
        gps_delta(i,1) = gps_delta(i-1,1) + gps_ddelta(i-1,1) * dt;
        gps_delta(i,2) = gps_delta(i-1,2) + gps_ddelta(i-1,2) * dt;
    end
    gps_ddelta(i,1) = - mu * gps_ddelta(i,1) + sqrt(2 * (sigma*cos(gps_angle))^2 * mu) * normrnd(0,1);
    gps_ddelta(i,2) = - mu * gps_ddelta(i,2) + sqrt(2 * (sigma*sin(gps_angle))^2 * mu) * normrnd(0,1);

    if rem(i, 100) == 0 || i == 1
        gps_pos_errorX = gps_pos_error * sin(Time(i)/250);
        gps_pos_errorY = gps_pos_error * cos(Time(i)/350);
        gps_pos(i,1) = X(i) + lsx*cos(Heading(i)) - lsy*sin(Heading(i)) ...
            + gps_pos_errorX + normrnd(0, gps_pos_noise);
        gps_pos(i,2) = Y(i) + lsx*sin(Heading(i)) + lsy*cos(Heading(i)) ...
            + gps_pos_errorY + normrnd(0, gps_pos_noise);
        % GPS velocity errors
        gps_vel_errorX = gps_vel_error * cos(Time(i)/150);
        gps_vel_errorY = gps_vel_error * sin(Time(i)/80);
        gps_vel(i,1) = V(i)*cos(Heading(i) + Kappa(i)) ...
            + Anr(i) * (lsx*cos(Heading(i)) - lsy*sin(Heading(i))) ...
            + gps_vel_errorX + normrnd(0, gps_vel_noise);
        gps_vel(i,2) = V(i)*sin(Heading(i) + Kappa(i)) ...
            + Anr(i) * (lsx*sin(Heading(i)) + lsy*cos(Heading(i))) ...
            + gps_vel_errorY + normrnd(0, gps_vel_noise);
        
        % Scientific GNSS errors
        gps_pos_sci(i,1) = X(i) + delta*cos(gps_angle) + normrnd(0,sigma_w) + gps_delta(i,1);
        gps_pos_sci(i,2) = Y(i) + delta*sin(gps_angle) + normrnd(0,sigma_w) + gps_delta(i,2);
        % Antenna displacement
        gps_pos_sci(i,1) = gps_pos_sci(i,1) + lsx*cos(Heading(i)) - lsy*sin(Heading(i));
        gps_pos_sci(i,2) = gps_pos_sci(i,2) + lsx*sin(Heading(i)) + lsy*cos(Heading(i));
    else
        gps_pos(i,:) = gps_pos(i-1,:);
        gps_vel(i,:) = gps_vel(i-1,:);
        gps_pos_sci(i,1) = gps_pos_sci(i-1,1);
        gps_pos_sci(i,2) = gps_pos_sci(i-1,2);
    end
    % Gyroscope errors
    gyro_anr(i) = Anr(i) + gyro_bias + normrnd(0, gyro_noise);
    %% Baseline model

end



figure('Name', 'Robot trajectory');
plot(gps_pos_sci(:,2), gps_pos_sci(:,1), 'Color', [0, 0.8, 0], 'LineWidth', 2.0)
% plot(Y,X, 'k', 'LineWidth', 2.0)
axis equal
grid on
hold on
plot(Y,X, 'k', 'LineWidth', 2.0)
xlabel 'Y, м'
ylabel 'X, м'
legend 'Модель измерений СНС' 'Заданная траектория'

figure('Name', 'GPS location measurement compare');
plot(Time, gps_pos_sci(:,1) - X(:), 'b', 'LineWidth', 1.0)
grid on
hold on
plot(Time, Anr - mean(Anr), 'r', 'LineWidth', 1.0)
legend 'Модель ошибок' 'Реальные ошибки'
xlabel 'Время, с'
ylabel 'Ошибка измерения, м'

% figure('Name', 'Robot rotation velocity');
% plot(Time, gyro_anr, 'r', 'LineWidth', 1.0)
% grid on
% hold on
% plot(Time, Anr, 'b', 'LineWidth', 1.0)
% legend Anr Gyro
% 
% figure('Name', 'Robot velocity');
% plot(Time, V, 'b', 'LineWidth', 1.0)
% grid on
% hold on
% plot(Time, sqrt(gps_vel(:,1).^2 + gps_vel(:,2).^2), 'r', 'LineWidth', 1.0)
% legend V gpsVel
% 
% figure('Name', 'Encoder readings');
% plot(Time, odo_w(:,1), 'r', 'LineWidth', 1.0)
% grid on
% hold on
% plot(Time, odo_w(:,2), '--r', 'LineWidth', 1.0)
% plot(Time, w(:,2), '--b', 'LineWidth', 1.0)
% plot(Time, w(:,1), 'b', 'LineWidth', 1.0)
% % legend omega1 omega2 odo1
% 
% figure('Name', 'Encoder readings');
% plot(Time, odo_w(:,3), 'r', 'LineWidth', 1.0)
% grid on
% hold on
% plot(Time, odo_w(:,4), '--r', 'LineWidth', 1.0)
% plot(Time, w(:,4), '--b', 'LineWidth', 1.0)
% plot(Time, w(:,3), 'b', 'LineWidth', 1.0)
% % legend omega1 omega2 odo1
% 
% figure('Name', 'Gamma readings');
% 
% plot(Time, odo_gamma, 'r', 'LineWidth', 1.0)
% grid on
% hold on
% plot(Time, gamma(:,2), '--b', 'LineWidth', 1.0)
% plot(Time, gamma(:,1), 'b', 'LineWidth', 1.0)
% % plot(Time, odo_gamma(:,2), '--r', 'LineWidth', 1.0)
% legend gamma1 gamma2 odo
% 
% 
% figure('Name', 'GNSS Errors');
% plot(Time, Y, 'b', 'LineWidth', 1.0);
% hold on;
% grid on
% plot(Time, gps_pos(:,2), 'c', 'LineWidth', 1.0);
% plot(Time, gps_pos_sci(:,2), 'r', 'LineWidth', 1.0);



%%
gps_pos = gps_pos_sci;
clearvars -except Kappa odo_gamma Time odo_w gps_vel gps_pos gyro_anr X Y Heading Betta V Anr w gamma

