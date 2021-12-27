function [state_prime, params] = kalmanNav(state, sensors, params)
    
    dt = sensors.dt;
    odo_omega = sensors.omega; % wheels rotation rate
    odo_gamma = sensors.gamma; % front wheels rotation angle
    avs_dpsi = sensors.gyro;
    gps_x = sensors.gps_x;
    gps_y = sensors.gps_y;
    gps_dx = sensors.gps_dx;
    gps_dy = sensors.gps_dy;

    x = params.x;
    y = params.y;
    psi = params.heading;
    dpsi = params.angular_rate;
    vel = params.velocity; % velx vely
    gamma = params.gamma; % 2 angles
    betta = params.betta;
    omega = params.omega;
    
    
    lsx = -0.5;
    lsy = 0.1;
    lf = 0.4;
    lr = 0.4;
    lw = 0.6;
    rw = 0.254/2;
    
    
    %% Kalman Prediction
    F = zeros(16, 16);
    F(1:2,1:4) = [1, 0, dt, 0;
                  0, 1, 0, dt];
    F(5,1:6) = [0, 0, 0, 0, 1, dt];
    F(9:16,9:16) = eye(8);
    % vel_xg -----
    % psi
    F(3,5) = -0.25 * rw * (omega(1)*sin(betta(1) + gamma(1) + psi) + ...
        omega(2)*sin(betta(2) + gamma(2) + psi) + ...
        omega(3)*sin(betta(3) + psi) + omega(4)*sin(betta(4) + psi));
    % gamma 1,2
    F(3,7) = - 0.25 * omega(1) * rw * sin(betta(1) + gamma(1) + psi);
    F(3,8) = - 0.25 * omega(2) * rw * sin(betta(2) + gamma(2) + psi);
    % omega 1,2,3,4
    F(3,9) = 0.25 * rw * cos(betta(1) + gamma(1) + psi);
    F(3,10) = 0.25 * rw * cos(betta(2) + gamma(2) + psi);
    F(3,11) = 0.25 * rw * cos(betta(3) + psi);
    F(3,12) = 0.25 * rw * cos(betta(4) + psi);
    % betta 1,2,3,4
    F(3,13) = - 0.25 * omega(1) * rw * sin(betta(1) + gamma(1) + psi);
    F(3,14) = - 0.25 * omega(2) * rw * sin(betta(2) + gamma(2) + psi);
    F(3,15) = - 0.25 * omega(3) * rw * sin(betta(3) + psi);
    F(3,16) = - 0.25 * omega(4) * rw * sin(betta(4) + psi);
    % vel_yg -----
    % psi
    F(4,5) = 0.25 * rw * (omega(1)*cos(betta(1) + gamma(1) + psi) + ...
        omega(2)*cos(betta(2) + gamma(2) + psi) + ...
        omega(3)*cos(betta(3) + psi) + omega(4)*cos(betta(4) + psi));
    % gamma 1,2
    F(4,7) = 0.25 * omega(1) * rw * cos(betta(1) + gamma(1) + psi);
    F(4,8) = 0.25 * omega(2) * rw * cos(betta(2) + gamma(2) + psi);
    % omega 1,2,3,4
    F(4,9) = 0.25 * rw * sin(betta(1) + gamma(1) + psi);
    F(4,10) = 0.25 * rw * sin(betta(2) + gamma(2) + psi);
    F(4,11) = 0.25 * rw * sin(betta(3) + psi);
    F(4,12) = 0.25 * rw * sin(betta(4) + psi);
    % betta 1,2,3,4
    F(4,13) = 0.25 * omega(1) * rw * cos(betta(1) + gamma(1) + psi);
    F(4,14) = 0.25 * omega(2) * rw * cos(betta(2) + gamma(2) + psi);
    F(4,15) = 0.25 * omega(3) * rw * cos(betta(3) + psi);
    F(4,16) = 0.25 * omega(4) * rw * cos(betta(4) + psi);
    
    % dpsi ---
    V = sqrt(vel(1)^2 + vel(2)^2);
    B = atan2(vel(2), vel(1)) - psi;
    F(6,7) = 0.5 * V * cos(B) / (lf + lr) / cos(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2)))^2;
    F(6,8) = 0.5 * V * cos(B) / (lf + lr) / cos(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2)))^2;
    F(6,9) = 0.25 * rw * cos(B) / (lf + lr) * ( tan(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2))) - tan(0.5*(betta(3) + betta(4))) );
    F(6,10) = 0.25 * rw * cos(B) / (lf + lr) * ( tan(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2))) - tan(0.5*(betta(3) + betta(4))) );
    F(6,11) = 0.25 * rw * cos(B) / (lf + lr) * ( tan(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2))) - tan(0.5*(betta(3) + betta(4))) );
    F(6,12) = 0.25 * rw * cos(B) / (lf + lr) * ( tan(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2))) - tan(0.5*(betta(3) + betta(4))) );
    F(6,13) = 0.5 * V * cos(B) / (lf + lr) / cos(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2)))^2;
    F(6,14) = 0.5 * V * cos(B) / (lf + lr) / cos(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2)))^2;
    F(6,15) = 0.5 * V * cos(B) / (lf + lr) / cos(0.5*(betta(3) + betta(4)))^2;
    F(6,16) = 0.5 * V * cos(B) / (lf + lr) / cos(0.5*(betta(3) + betta(4)))^2;
    
    % dgamma1 ---
    F(7,7) = 0.5 - 2 * lw / (lf + lr) * 0.5;% * (gamma(1) + gamma(2)));
    F(7,8) = 0.5 - 2 * lw / (lf + lr) * 0.5;% * (gamma(1) + gamma(2)));
    
%     cosf = 2 * cos(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2)))^2;
%     cosr = 2 * cos(0.5*(betta(3) + betta(4)))^2;
%     tanfr = tan(0.5*(gamma(1) + gamma(2) + betta(1) + betta(2))) - tan(0.5*(betta(3) + betta(4)));
%     % dpsi
%     F(7,6) = (lf + lr) * cosf / V / cos(B);
%     % dgamma2
%     F(7,8) = -1;
%     % domega
%     F(7,9) = - 0.25 * rw / V * cosf * tanfr;
%     F(7,10) = - 0.25 * rw / V * cosf * tanfr;
%     F(7,11) = - 0.25 * rw / V * cosf * tanfr;
%     F(7,12) = - 0.25 * rw / V * cosf * tanfr;
%     % dbetta
%     F(7,13) = -1;
%     F(7,14) = -1;
%     F(7,15) = cosf / cosr;
%     F(7,16) = cosf / cosr;
    
    % dgamma2 ---
    F(8,7) = 0.5 + 2 * lw / (lf + lr) * 0.5;% * (gamma(1) + gamma(2)));
    F(8,8) = 0.5 + 2 * lw / (lf + lr) * 0.5;% * (gamma(1) + gamma(2)));
%     % dpsi
%     F(8,6) = (lf + lr) * cosf / V / cos(B);
%     % dgamma1
%     F(8,7) = -1;
%     % domega
%     F(8,9) = - 0.25 * rw / V * cosf * tanfr;
%     F(8,10) = - 0.25 * rw / V * cosf * tanfr;
%     F(8,11) = - 0.25 * rw / V * cosf * tanfr;
%     F(8,12) = - 0.25 * rw / V * cosf * tanfr;
%     % dbetta
%     F(8,13) = -1;
%     F(8,14) = -1;
%     F(8,15) = cosf / cosr;
%     F(8,16) = cosf / cosr;



    U = [state.epos, state.epos, state.evel, state.evel, state.epsi, state.edpsi, ...
        state.egamma, state.egamma, state.ew, state.ew, state.ew, state.ew, ...
        state.eb, state.eb, state.eb, state.eb]';
    Q = diag(U);
    
    Xprime = F * state.X + U;
    Pprime = F * state.P * F' + Q;
    
    %% Kalamn Correction GYRO
    if (sensors.gyro_update == 1)
        Z = [avs_dpsi - dpsi];
        H = zeros(1,16);
        H(6) = -1;
        R = [sensors.egyro];
        I = eye(16);
        Y = Z - H * Xprime;
        S = H * Pprime * H' + R;
        K = Pprime * H' * inv(S);
        Xprime = Xprime + K * Y;
        Pprime = (I - K * H) * Pprime;
    end
    
    %% Kalman Correction GPS
    if (sensors.gps_update == 1)
        Z = [gps_x - x - lsx * cos(psi) + lsy + sin(psi);
            gps_y - y - lsx * sin(psi) - lsy * cos(psi);
            gps_dx - vel(1);
            gps_dy - vel(2)];
        R = [sensors.egps_pos, sensors.egps_pos, sensors.egps_vel, sensors.egps_vel];
        H = zeros(4,16);
        H(1:2,1:2) = eye(2);
        H(1,5) = -(lsx * sin(psi) + lsy * cos(psi));
        H(2,5) = lsx * cos(psi) - lsy * sin(psi);
        H(3:4,3:4) = eye(2);
        H(3,5) = - (dpsi * lsx * sin(psi) + dpsi * lsy * cos(psi));
        H(3,6) = lsx * cos(psi) - lsy * sin(psi);
        H(4,5) = dpsi * lsx * cos(psi) - dpsi * lsy * sin(psi);
        H(4,6) = lsx * sin(psi) + lsy * cos(psi);
        I = eye(16);
        Y = Z - H * Xprime;
        S = H * Pprime * H' + R;
        K = Pprime * H' * inv(S);
        Xprime = Xprime + K * Y;
        Pprime = (I - K * H) * Pprime;
    end
    %%
    state_prime = state;
    state_prime.P = Pprime;
    state_prime.X = Xprime;
    
    
    V = 0.25 * rw * (omega(1) + omega(2) + omega(3) + omega(4));
    Vxb = 0.25 * rw * (omega(1)*cos(betta(1)+gamma(1)) +...
        omega(2)*cos(betta(2)+gamma(2))+...
        omega(3)*cos(betta(3))+...
        omega(4)*cos(betta(4)));
    Vyb = 0.25 * rw * (omega(1)*sin(betta(1)+gamma(1)) +...
        omega(2)*sin(betta(2)+gamma(2))+...
        omega(3)*sin(betta(3))+...
        omega(4)*sin(betta(4)));
    B = atan2(Vyb, Vxb);
    
    params.x = params.x + params.velocity(1)*dt;% - Xprime(1);
    params.y = params.y + params.velocity(2)*dt;% - Xprime(2);
%     params.velocity(1) = V * cos(params.heading);% - Xprime(3);
%     params.velocity(2) = V * sin(params.heading);% - Xprime(4);
    params.velocity(1) = Vxb * cos(params.heading) - Vyb * sin(params.heading);% - Xprime(3);
    params.velocity(2) = Vxb * sin(params.heading) + Vyb * cos(params.heading);
    % TODO: figure out whats the difference between substracting X(5) at
    % each step, or substracting two arrays in the end
    params.heading = params.heading + params.angular_rate * dt;% + Xprime(5);
%     params.heading = angle_lim(params.heading, [-2*pi, 2*pi]);
    params.angular_rate = V * cos(B) * (tan(0.5 * (gamma(1) + gamma(2) + betta(1) + betta(2))) - tan(0.5*(betta(3) + betta(4)))) / (lr + lf);% - Xprime(6);
%     params.angular_rate = V * cos(B) * tan(0.5 * (gamma(1) + gamma(2))) / (lr + lf) - Xprime(6);
    params.gamma(1) = odo_gamma - odo_gamma^2 * lw / (lf + lr);% - Xprime(7);
    params.gamma(2) = odo_gamma + odo_gamma^2 * lw / (lf + lr);% - Xprime(8);
    params.omega(1) = odo_omega(1);% - Xprime(9);
    params.omega(2) = odo_omega(2);% - Xprime(10);
    params.omega(3) = odo_omega(3);% - Xprime(11);
    params.omega(4) = odo_omega(4);% - Xprime(12);
    params.betta(1) = params.betta(1);% - Xprime(13);
    params.betta(2) = params.betta(2);% - Xprime(14);
    params.betta(3) = params.betta(3);% - Xprime(15);
    params.betta(4) = params.betta(4);% - Xprime(16);
end

