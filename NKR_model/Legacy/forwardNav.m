function state_prime = forwardNav(state, sensors, update)
    
    dt = 0.01;
    heading = state.X(3);
    tangf = tan(sensors.gammaf);
    tangr = tan(sensors.gammar);
    lsx = 0;%0.15;
    lsy = 0;%0.05;
    lf = 0.4;
    lr = 0.4;
    lw = 0.3;
    rw = 0.1;
    
    %% Prediction
    if abs(state.X(5)) <= 0.01
        A1 = 0; A2 = 0; A3 = 0; A4 = 0;
        B = 1 / rw;
    else
        R = abs(state.X(4)/state.X(5));
        A1 = sign(state.X(5))*sqrt(lf^2 + (R+sign(state.X(5))*lw/2)^2)/rw;
        A2 = sign(state.X(5))*sqrt(lf^2 + (R-sign(state.X(5))*lw/2)^2)/rw;
        A3 = sign(state.X(5))*sqrt(lr^2 + (R+sign(state.X(5))*lw/2)^2)/rw;
        A4 = sign(state.X(5))*sqrt(lr^2 + (R-sign(state.X(5))*lw/2)^2)/rw;
        B = 0;
    end
%     X = [X Y Heading V dHeading w1 w2 w3 w4];
%         X  Y  H.                    V      dH.   w1    w2    w3    w4
    F = [ 1, 0, 0,       cos(heading)*dt,     0,    0,    0,    0,    0; % X
          0, 1, 0,       sin(heading)*dt,     0,    0,    0,    0,    0; % Y
          0, 0, 1,                     0,    dt,    0,    0,    0,    0; % Heading
          0, 0, 0,                     0,     0, rw/4, rw/4, rw/4, rw/4; % V
          0, 0, 0, (tangf-tangr)/(lf+lr),     0,    0,    0,    0,    0; % dHeading
          0, 0, 0,                   B*0.75,  A1*0.75,  0.25,    0,    0,    0; % w1
          0, 0, 0,                   B*0.75,  A2*0.75,    0,  0.25,    0,    0; % w2
          0, 0, 0,                   B*0.75,  A3*0.75,    0,    0,  0.25,    0; % w3
          0, 0, 0,                   B*0.75,  A4*0.75,    0,    0,    0,  0.25];% w4
    % Степень доверия предсказаниям (чем меньше коэф. - тем больше ему
    % доверяем)
    U = [2e-5 2e-5 1e-5 1e-2 1e-3 1e-2 1e-2 1e-2 1e-2]';

    
    
    Xp = F*state.X + U;
    Pp = F * state.P * F' + diag(U);
    
    P = Pp;
    X = Xp;
    
    %% Correction
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
    Xp = X + K*y; % X'
    Pp = (eye(9) - K*H)*P; % P'
    
    
    state_prime.P = Pp;
    state_prime.X = Xp;
    
end

