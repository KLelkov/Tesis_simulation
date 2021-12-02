function state_prime = kalmanNav(state, sensors, params)
    
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
    F(7:16,7:16) = eye(10);
    % vel_x
    F(3,7) = - 0.25 * omega(1) * rw * sin(betta(1) + gamma(1) + psi);
    F(3,8) = - 0.25 * omega(2) * rw * sin(betta(2) + gamma(2) + psi);
    F(3,9) = 0.25 * rw * cos(betta(1) + gamma(1) + psi);
    F(3,10) = 0.25 * rw * cos(betta(2) + gamma(2) + psi);
    F(3,11) = 0.25 * rw * cos(betta(3) + psi);
    F(3,12) = 0.25 * rw * cos(betta(4) + psi);
    F(3,13) = - 0.25 * omega(1) * rw * sin(betta(1) + gamma(1) + psi);
    F(3,14) = - 0.25 * omega(2) * rw * sin(betta(2) + gamma(2) + psi);
    F(3,15) = - 0.25 * omega(3) * rw * sin(betta(3) + psi);
    F(3,16) = - 0.25 * omega(4) * rw * sin(betta(4) + psi);
    % vel_y
    F(4,7) = - 0.25 * omega(1) * rw * cos(betta(1) + gamma(1) + psi);
    F(4,8) = - 0.25 * omega(2) * rw * cos(betta(2) + gamma(2) + psi);
    F(4,9) = 0.25 * rw * sin(betta(1) + gamma(1) + psi);
    F(4,10) = 0.25 * rw * sin(betta(2) + gamma(2) + psi);
    F(4,11) = 0.25 * rw * sin(betta(3) + psi);
    F(4,12) = 0.25 * rw * sin(betta(4) + psi);
    F(4,13) = 0.25 * omega(1) * rw * cos(betta(1) + gamma(1) + psi);
    F(4,14) = 0.25 * omega(2) * rw * cos(betta(2) + gamma(2) + psi);
    F(4,15) = 0.25 * omega(3) * rw * cos(betta(3) + psi);
    F(4,16) = 0.25 * omega(4) * rw * cos(betta(4) + psi);
    % dpsi
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
    F(6,7) = V * cos(B) / (lf + lr) / cos(0.5*(gamma(1) + gamma(2)))^2 / 2;
    F(6,8) = V * cos(B) / (lf + lr) / cos(0.5*(gamma(1) + gamma(2)))^2 / 2;
    F(6,9) = 0.25 * rw * cos(B) / (lf + lr) * tan(0.5*(gamma(1) + gamma(2)));
    F(6,10) = 0.25 * rw * cos(B) / (lf + lr) * tan(0.5*(gamma(1) + gamma(2)));
    F(6,11) = 0.25 * rw * cos(B) / (lf + lr) * tan(0.5*(gamma(1) + gamma(2)));
    F(6,12) = 0.25 * rw * cos(B) / (lf + lr) * tan(0.5*(gamma(1) + gamma(2)));
    
    U = [state.epos, state.epos, state.evel, state.evel, state.epsi, state.edpsi, ...
        state.egamma, state.egamma, state.ew, state.ew, state.ew, state.ew, ...
        state.eb, state.eb, state.eb, state.eb];
    Q = diag(U);
    
    Xprime = F * state.X + U;
    Pprime = F * state.P * F' + Q;
    
    %% Kalamn Correction GYRO
    if (sensors.gyro_update == 1)
        Z = [avs_dpsi - dpsi];
        H = zeros(1,16);
        H(5) = -1;
        R = [sensros.egyro];
        I = eye(16);
        Y = Z - H * Xprime;
        S = H * P * H' + R;
        K = P * H' * inv(S);
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
        H(3,5) = - (dpsi * lsx * sinz(psi) + dpsi * lsy * cos(psi));
        H(3,6) = lsx * cos(psi) - lsy * sin(psi);
        H(4,5) = dpsi * lsx * cos(psi) - dpsi * lsy * sin(psi);
        H(4,6) = lsx * sin(psi) + lsy * cos(psi);
        I = eye(16);
        Y = Z - H * Xprime;
        S = H * P * H' + R;
        K = P * H' * inv(S);
        Xprime = Xprime + K * Y;
        Pprime = (I - K * H) * Pprime;
    end
    
    state_prime = state;
    state_prime.P = Pprime;
    state_prime.X = Xprime;
    
    params.x = params.x - Xprime(1);
    params.y = params.y - Xprime(2);
    params.vel(1) = Vxb - Xprime(3);
    params.vel(2) = Vyb - Xprime(4);
    params.heading = params.heading + params.angular_rate * dt - Xprime(5);
    params.angular_rate = V * cos(B) * tan(0.5 * (gamma(1) + gamma(2))) / (lr + lf) - Xprime(6);
    params.gamma(1) = odo_gamma - odo_gamma^2 * lw / (lf + lr) - Xprime(7);
    params.gamma(2) = odo_gamma + odo_gamma^2 * lw / (lf + lr) - Xprime(8);
    params.omega(1) = odo_omega(1) - Xprime(9);
    params.omega(2) = odo_omega(2) - Xprime(10);
    params.omega(3) = odo_omega(3) - Xprime(11);
    params.omega(4) = odo_omega(4) - Xprime(12);
    params.betta(1) = params.betta(1) - Xprime(13);
    params.betta(2) = params.betta(2) - Xprime(14);
    params.betta(3) = params.betta(3) - Xprime(15);
    params.betta(4) = params.betta(4) - Xprime(16);
end

