function [X, Y, Heading] = test_navigation(time, gamma, Gyro, gps_pos, gps_vel, omega)
    len = length(time);
    sensors.dt = 0.01;

    X = zeros(len, 1);
    Y = zeros(len, 1);
    Heading = zeros(len, 1);
    
    
    % init sensors struct
    sensors.odo_error = 1e-1;
    sensors.egyro = 2e-1;
    sensors.egps_pos = 1e0;
    sensors.egps_vel = 8e-1;
    
    %init Kalman state struct
    kalman_state.X = zeros(16,1);
    kalman_state.X(3) = 0; % set initial heading
    kalman_state.P = diag([100 100 2 2 3 1 1 1 10 10 10 10 1 1 1 1]);
    kalman_state.epos = 2e-5;
    kalman_state.epsi = 1e-4;
    kalman_state.evel = 3e-3;
    kalman_state.edpsi = 8e-4;
    kalman_state.ew = 1e-2;%8e-3;
    kalman_state.eb = 1e-2;%8e-3;
    kalman_state.egamma = 1e-1;%8e-3;

    
    
    % init nav params struct
    nav_params.x = 0;
    nav_params.y = 0;
    nav_params.velocity(1) = 0;
    nav_params.velocity(2) = 0;
    nav_params.heading = 0;
    nav_params.angular_rate = 0;
    nav_params.gamma(1) = 0;
    nav_params.gamma(2) = 0;
    nav_params.omega(1) = 0;
    nav_params.omega(2) = 0;
    nav_params.omega(3) = 0;
    nav_params.omega(4) = 0;
    nav_params.betta(1) = 0;
    nav_params.betta(2) = 0;
    nav_params.betta(3) = 0;
    nav_params.betta(4) = 0;
    
    for i = 1:500
       
        sensors.gamma = gamma(i);
        sensors.omega = omega(i,:);

        sensors.gps_x = gps_pos(i,1);
        sensors.gps_y = gps_pos(i,2);
        sensors.gps_dx = gps_vel(i,1);
        sensors.gps_dy = gps_vel(i,2);
        sensors.gps_update = 0;
        if i > 1 && gps_pos(i) ~= gps_pos(i-1)
            
            sensors.gps_update = 1;
        end
        
        sensors.gyro = Gyro(i);
        sensors.gyro_update = 1;
        
        [kalman_state, nav_params] = kalmanNav(kalman_state, sensors, nav_params);
        X(i) = nav_params.x;
        Y(i) = nav_params.x;
        Heading(i) = nav_params.heading;
        
    end
    figure
    plot(Y,X);
    axis equal
    grid on
    figure;
    plot(Heading)
    grid on
end