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
    
    %% Kalman Correction ODO
    
    
    %% Kalman Correction GPS
    
    %% Prediction
    % Motion transform (apply system dynamic F)


    % Степень доверия предсказаниям (чем меньше коэф. - тем больше ему
    % доверяем)
    
    
    % Apply prediction update to every column of Xs
    for j = 1:n
        Xs(1,j) = Xs(1,j) + Xs(4,j)*cos(Xs(3,j))*dt; % X
        Xs(2,j) = Xs(2,j) + Xs(4,j)*sin(Xs(3,j))*dt; % Y
        Xs(3,j) = Xs(3,j) + Xs(5,j)*dt; % Heading
        Xs(4,j) = rw / 4 * ( Xs(6,j) + Xs(7,j) + Xs(8,j) + Xs(9,j) ); % V
        Xs(5,j) = (tangf-tangr)/(lf+lr) * Xs(4,j); % dHeading
%         Xs(5,j) = rw / (2 * lw) * (Xs(6,j) - Xs(7,j) + Xs(8,j) - Xs(9,j));
        Xs(6,j) = Xs(6,j); % w1
        Xs(7,j) = Xs(7,j); % w2
        Xs(8,j) = Xs(8,j); % w3
        Xs(9,j) = Xs(9,j); % w4
    end
    
    %% Reverse unscented transform
    % Derive Xprime as weighted mean
    for i = 1:dim
        Xprime(i) = 0;
        for j = 1:n
            Xprime(i) = Xprime(i) + Xs(i,j)*Ws(1,j);
        end
    end
    % Derive Pprime as weighted covariance
    for j = 1:n
        disp(j)
        a = Ws(2,j) * ((Xs(:,j) - Xprime) * (Xs(:,j) - Xprime)');
        disp(a)
        Pprime = Pprime + a;
    end
    Pprime = Pprime + diag(U);


    P = Pprime;
    X = Xprime;
    
    %% Correction
%     
%     % Apply H to Xs to get predicted measurement Z
%     if update.gyro == 1 && update.gps == 1
%         dimz = 9;
%         Zs = zeros(dimz, n);
%         Zhat = zeros(dimz, 1);
%         Z = [sensors.gps_x;
%                 sensors.gps_y;
%                 sensors.gps_dx;
%                 sensors.gps_dy;
%                 sensors.gyro;
%                 sensors.w1;
%                 sensors.w2;
%                 sensors.w3;
%                 sensors.w4];
%         % Дисперсии ошибок датчиков (паспортные данные)
%         R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
%         for j = 1:n
%             Zs(1,j) = Xs(1,j) + lsx*cos(Xs(3,j)) - lsy*sin(Xs(3,j));
%             Zs(2,j) = Xs(2,j) + lsx*sin(Xs(3,j)) + lsy*cos(Xs(3,j));
%             Zs(3,j) = Xs(4,j)*cos(Xs(3,j)) + Xs(5,j)*(lsx*cos(Xs(3,j)) - lsy*sin(Xs(3,j)));
%             Zs(4,j) = Xs(4,j)*sin(Xs(3,j)) + Xs(5,j)*(lsx*sin(Xs(3,j)) + lsy*cos(Xs(3,j)));
%             Zs(5,j) = Xs(5,j);
%             Zs(6,j) = Xs(6,j);
%             Zs(7,j) = Xs(7,j);
%             Zs(8,j) = Xs(8,j);
%             Zs(9,j) = Xs(9,j);
%         end
%     elseif update.gps == 1 % never happens (gps always comes with gyro)
%         dimz = 8;
%         Zs = zeros(dimz, n);
%         Zhat = zeros(dimz, 1);
%         Z = [sensors.gps_x;
%                 sensors.gps_y;
%                 sensors.gps_dx;
%                 sensors.gps_dy;
%                 sensors.w1;
%                 sensors.w2;
%                 sensors.w3;
%                 sensors.w4];
%         % Дисперсии ошибок датчиков (паспортные данные)
%         R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
%         for j = 1:n
%             Zs(1,j) = Xs(1,j) + lsx*cos(Xs(3,j)) - lsy*sin(Xs(3,j));
%             Zs(2,j) = Xs(2,j) + lsx*sin(Xs(3,j)) + lsy*cos(Xs(3,j));
%             Zs(3,j) = Xs(4,j)*cos(Xs(3,j)) + Xs(5,j)*(lsx*cos(Xs(3,j)) - lsy*sin(Xs(3,j)));
%             Zs(4,j) = Xs(4,j)*sin(Xs(3,j)) + Xs(5,j)*(lsx*sin(Xs(3,j)) + lsy*cos(Xs(3,j)));
%             Zs(5,j) = Xs(6,j);
%             Zs(6,j) = Xs(7,j);
%             Zs(7,j) = Xs(8,j);
%             Zs(8,j) = Xs(9,j);
%         end
%     elseif update.gyro == 1
%         dimz = 5;
%         Zs = zeros(dimz, n);
%         Zhat = zeros(dimz, 1);
%         Z = [ sensors.gyro;
%                 sensors.w1;
%                 sensors.w2;
%                 sensors.w3;
%                 sensors.w4];
%         % Дисперсии ошибок датчиков (паспортные данные)
%         R = diag([sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
%         for j = 1:n
%             Zs(1,j) = Xs(5,j);
%             Zs(2,j) = Xs(6,j);
%             Zs(3,j) = Xs(7,j);
%             Zs(4,j) = Xs(8,j);
%             Zs(5,j) = Xs(9,j);
%         end
%     else
%         dimz = 4;
%         Zs = zeros(dimz, n);
%         Zhat = zeros(dimz, 1);
%         Z = [ sensors.w1;
%                 sensors.w2;
%                 sensors.w3;
%                 sensors.w4];
%         % Дисперсии ошибок датчиков (паспортные данные)
%         R = diag([sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
%         for j = 1:n
%             Zs(1,j) = Xs(6,j);
%             Zs(2,j) = Xs(7,j);
%             Zs(3,j) = Xs(8,j);
%             Zs(4,j) = Xs(9,j);
%         end
%     end
%     
%     % Derive Zhat
%     for i = 1:dimz
%         Zhat(i) = 0;
%         for j = 1:n
%             Zhat(i) = Zhat(i) + Zs(i,j)*Ws(1,j);
%         end
%     end
%     
%     % Matrix S
%     S = zeros(dimz, dimz);
%     for j = 1:n
%         S = S + Ws(2,j) * ((Zs(:,j) - Zhat) * (Zs(:,j) - Zhat)');
%     end
%     S = S + diag(R);
%     
%     % Matrix Sigma (x,z)
%     Sig = zeros(dim, dimz);
%     for j = 1:n
%         Sig = Sig + Ws(2,j) * ((Xs(:,j) - Xprime) * (Zs(:,j) - Zhat)');
%     end
%     K = Sig*inv(S);
%     Xprime = X + K*(Z - Zhat);
%     Pprime = P - K*S*K';
    
%%
    heading = X(3);
    xproj = lsx*cos(heading) - lsy*sin(heading);
    yproj = lsx*sin(heading) + lsy*cos(heading);
    if update.gyro == 1 && update.gps == 1
        %     X  Y  H.            V     dH. w1 w2 w3 w4
        H = [ 1, 0, 0,            0,     0, 0, 0, 0, 0; % Xs
              0, 1, 0,            0,     0, 0, 0, 0, 0; % Ys
              0, 0, 0, cos(heading), xproj, 0, 0, 0 ,0; % dXs
              0, 0, 0, sin(heading), yproj, 0, 0, 0, 0; % dYs
              0, 0, 0,            0,     1, 0, 0, 0, 0; % dHeading_gyro
              0, 0, 0,            0,     0, 1, 0, 0, 0; % odo1
              0, 0, 0,            0,     0, 0, 1, 0, 0; % odo2
              0, 0, 0,            0,     0, 0, 0, 1, 0; % odo3
              0, 0, 0,            0,     0, 0, 0, 0, 1];% odo4

        Z = [sensors.gps_x - xproj;
            sensors.gps_y - yproj;
            sensors.gps_dx;
            sensors.gps_dy;
            sensors.gyro;
            sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    elseif update.gyro == 1
        %     X  Y     H.             V    dH. w1 w2 w3 w4
        H = [ 0, 0,     0,            0,     1, 0, 0, 0, 0; % dHeading_gyro
              0, 0,     0,            0,     0, 1, 0, 0, 0; % odo1
              0, 0,     0,            0,     0, 0, 1, 0, 0; % odo2
              0, 0,     0,            0,     0, 0, 0, 1, 0; % odo3
              0, 0,     0,            0,     0, 0, 0, 0, 1];% odo4

        Z = [sensors.gyro;
            sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    elseif update.gps == 1 % never happens (gps always comes with gyro)
        %     X  Y  H.            V     dH. w1 w2 w3 w4
        H = [ 1, 0, 0,            0,     0, 0, 0, 0, 0; % Xs
              0, 1, 0,            0,     0, 0, 0, 0, 0; % Ys
              0, 0, 0, cos(heading), xproj, 0, 0, 0 ,0; % dXs
              0, 0, 0, sin(heading), yproj, 0, 0, 0, 0; % dYs
              0, 0, 0,            0,     0, 1, 0, 0, 0; % odo1
              0, 0, 0,            0,     0, 0, 1, 0, 0; % odo2
              0, 0, 0,            0,     0, 0, 0, 1, 0; % odo3
              0, 0, 0,            0,     0, 0, 0, 0, 1];% odo4

        Z = [sensors.gps_x - xproj;
            sensors.gps_y - yproj;
            sensors.gps_dx;
            sensors.gps_dy;
            sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    else % only odometry is available (may happen if gyro measurements dont come at every simulation step)
        %     X  Y     H.             V    dH. w1 w2 w3 w4
        H = [ 0, 0,     0,            0,     0, 1, 0, 0, 0; % odo1
              0, 0,     0,            0,     0, 0, 1, 0, 0; % odo2
              0, 0,     0,            0,     0, 0, 0, 1, 0; % odo3
              0, 0,     0,            0,     0, 0, 0, 0, 1];% odo4

        Z = [sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    end
    
    
    y = Z - H*X; % невязка
    S = H * P * H' + R; 
    K = P * H' * inv(S);
    Xprime = X + K*y; % X'
    Pprime = (eye(9) - K*H)*P; % P'
    
    state_prime = state;
    state_prime.P = Pprime;
    state_prime.X = Xprime;
    
end

