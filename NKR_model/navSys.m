function [robot_prime, state_prime] = navSys(robot, state, sensors, update)
    % update.gyro = 1 == gyro update
    % update.gps = 1 == gps update
    dt = sensors.dt;
    
    
    
%     X = [state.sx, state.sy, state.sheading, state.sw1, state.sw2, state.sw3, state.sw4];
    A = robot.rw / 4;
    B = robot.rw * cos(robot.betta) * (tan(robot.gammaf) - tan(robot.gammar)) / (4*(robot.lf + robot.lr));
    C = 1/robot.rw + robot.lw*cos(robot.betta) * (tan(robot.gammaf) - tan(robot.gammar)) / (2*robot.rw*(robot.lf + robot.lr));
    D = 1/robot.rw - robot.lw*cos(robot.betta) * (tan(robot.gammaf) - tan(robot.gammar)) / (2*robot.rw*(robot.lf + robot.lr));
    
    F = [0, 0, -robot.v*sin(robot.heading+robot.betta), A*cos(robot.heading+robot.betta), A*cos(robot.heading+robot.betta), A*cos(robot.heading+robot.betta), A*cos(robot.heading+robot.betta);
        0, 0, robot.v*cos(robot.heading+robot.betta), A*sin(robot.heading+robot.betta), A*sin(robot.heading+robot.betta), A*sin(robot.heading+robot.betta), A*sin(robot.heading+robot.betta);
        0, 0, 0, B, B, B, B;
        0, 0, 0, C/dt, C/dt, C/dt, C/dt;
        0, 0, 0, D/dt, D/dt, D/dt, D/dt;
        0, 0, 0, C/dt, C/dt, C/dt, C/dt;
        0, 0, 0, D/dt, D/dt, D/dt, D/dt];
    % Степень доверия предсказаниям (чем меньше коэф. - тем больше ему
    % доверяем)
    U = diag([1e-1 1e-1 1e-2 1e-1 1e-1 1e-1 1e-1]);
    
    % Prediction
    Xp = F*state.X + U;
    Pp = F * state.P * F';
    
    P = Pp;
    X = Xp;
    
    E = B * (robot.lsx*cos(robot.heading) - robot.lsy*sin(robot.heading));
    G = B * (robot.lsx*sin(robot.heading) + robot.lsy*cos(robot.heading));
    
    if update.gyro == 1 && update.gps == 1 
        H = [0, 0, -(robot.lsx*sin(robot.heading) + robot.lsy*cos(robot.heading)), 0, 0, 0, 0;
        0, 0, (robot.lsx*cos(robot.heading) - robot.lsy*sin(robot.heading)), 0, 0, 0, 0;
        0, 0, -robot.dheading*(robot.lsx*sin(robot.heading) + robot.lsy*cos(robot.heading)), E, E, E, E;
        0, 0, robot.dheading*(robot.lsx*cos(robot.heading) - robot.lsy*sin(robot.heading)), G, G, G, G;
        0, 0, 0, B, B, B, B];

        Z = [sensors.gps_x - robot.x - robot.lsx*cos(robot.heading) + robot.lsy*sin(robot.heading);
            sensors.gps_y - robot.y - robot.lsx*sin(robot.heading) - robot.lsy*cos(robot.heading);
            sensors.gps_dx - robot.v*cos(robot.heading + robot.betta);
            sensors.gps_dy - robot.v*sin(robot.heading + robot.betta);
            sensors.gyro - robot.dheading];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.gyro_error]);
    elseif update.gyro == 1
        H = [0, 0, 0, B, B, B, B];

        Z = [sensors.gyro - robot.dheading];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.gyro_error]);
    elseif update.gps == 1
        H = [0, 0, -(robot.lsx*sin(robot.heading) + robot.lsy*cos(robot.heading)), 0, 0, 0, 0;
        0, 0, (robot.lsx*cos(robot.heading) - robot.lsy*sin(robot.heading)), 0, 0, 0, 0;
        0, 0, -robot.dheading*(robot.lsx*sin(robot.heading) + robot.lsy*cos(robot.heading)), E, E, E, E;
        0, 0, robot.dheading*(robot.lsx*cos(robot.heading) - robot.lsy*sin(robot.heading)), G, G, G, G];

        Z = [sensor.gps_x - robot.x - robot.lsx*cos(robot.heading) + robot.lsy*sin(robot.heading);
            sensor.gps_y - robot.y - robot.lsx*sin(robot.heading) - robot.lsy*cos(robot.heading);
            sensor.gps_dx - robot.v*cos(robot.heading + robot.betta);
            sensor.gps_dy - robot.v*sin(robot.heading + robot.betta)];
        % Дисперсии ошибок датчиков (паспортные данные)
        R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel]);
    else
        state_prime.P = Pp;
        state_prime.X = Xp;

        robot_prime = robot;
        robot_prime.heading = robot.heading + robot.dheading*dt - state_prime.X(3);
        robot_prime.x = robot.x + robot.v*cos(robot_prime.heading + robot_prime.betta)*dt  - state_prime.X(1);
        robot_prime.y = robot.y + robot.v*sin(robot_prime.heading + robot_prime.betta)*dt  - state_prime.X(2);
        robot_prime.w1 = robot.w1 - state_prime.X(4);
        robot_prime.w2 = robot.w2 - state_prime.X(5);
        robot_prime.w3 = robot.w3 - state_prime.X(6);
        robot_prime.w4 = robot.w4 - state_prime.X(7);

        robot_prime.v = robot.rw/4 * (robot_prime.w1 + robot_prime.w2 + robot_prime.w3 + robot_prime.w4);
        robot_prime.dheading = robot_prime.v * cos(robot.betta) / (robot.lf + robot.lr) * (tan(robot.gammaf) - tan(robot.gammar));
        return;
    end
    % Correction
    y = Z - H*X; % невязка
    S = H * P * H' + R; 
    K = P * H' * inv(S);
    Xp = X + K*y; % X'
    Pp = (eye(7) - K*H)*P; % P'
    
    
    state_prime.P = Pp;
    state_prime.X = Xp;
    
    robot_prime = robot;
    robot_prime.heading = robot.heading + robot.dheading*dt - state_prime.X(3);
    robot_prime.x = robot.x + robot.v*cos(robot_prime.heading + robot_prime.betta)*dt  - state_prime.X(1);
    robot_prime.y = robot.y + robot.v*sin(robot_prime.heading + robot_prime.betta)*dt  - state_prime.X(2);
    robot_prime.w1 = robot.w1 - state_prime.X(4);
    robot_prime.w2 = robot.w2 - state_prime.X(5);
    robot_prime.w3 = robot.w3 - state_prime.X(6);
    robot_prime.w4 = robot.w4 - state_prime.X(7);
    
    robot_prime.v = robot.rw/4 * (robot_prime.w1 + robot_prime.w2 + robot_prime.w3 + robot_prime.w4);
    robot_prime.dheading = robot_prime.v * cos(robot.betta) / (robot.lf + robot.lr) * (tan(robot.gammaf) - tan(robot.gammar));
       
end

