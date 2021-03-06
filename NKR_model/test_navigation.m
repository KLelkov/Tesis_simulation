function [X, Y, Heading, Velocity, Rate, Betta, vect, Omega, Gamma] = test_navigation(time, gamma, Gyro, gps_pos, gps_vel, omega)
    len = length(time);
    sensors.dt = 0.01;

    X = zeros(len, 1);
    Y = zeros(len, 1);
    Heading = zeros(len, 1);
    Velocity = zeros(len, 2);
    Rate = zeros(len, 1);
    Betta = zeros(len, 4);
    Omega = zeros(len, 4);
    Gamma = zeros(len, 2);
    
    vect = zeros(len, 16);
%     velc = zeros(len, 1);
    
    
    % init sensors struct
    sensors.odo_error = 1e-1;
    sensors.egyro = 1e-2;
    sensors.egps_pos = 1e0;
    sensors.egps_vel = 8e-1;
    
    %init Kalman state struct
    kalman_state.X = zeros(16,1);
    kalman_state.X(3) = 0; % set initial heading
    kalman_state.P = diag([100 100 2 2 3 1 1 1 10 10 10 10 1 1 1 1]);
    kalman_state.epos = 2e-5;
    kalman_state.epsi = 1e-6;
    kalman_state.evel = 3e-3;
    kalman_state.edpsi = 2e-4;
    kalman_state.ew = 1e-6;%8e-3;
    kalman_state.eb = 5e-6;%8e-3;
    kalman_state.egamma = 2e-4;%8e-3;

    
    
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
    
    
    limit = 5000;
    for i = 1:limit
       
        sensors.gamma = gamma(i);
        sensors.omega = omega(i,:);

        sensors.gps_x = gps_pos(i,1);
        sensors.gps_y = gps_pos(i,2);
        sensors.gps_dx = gps_vel(i,1);
        sensors.gps_dy = gps_vel(i,2);
        sensors.gps_update = 0;
        if i > 1 && gps_pos(i) ~= gps_pos(i-1)
            
%             sensors.gps_update = 1;
        end
        
        sensors.gyro = Gyro(i);
        sensors.gyro_update = 1;
        
        [kalman_state, nav_params] = kalmanNav(kalman_state, sensors, nav_params);
        X(i) = nav_params.x;
        Y(i) = nav_params.y;
        Heading(i) = nav_params.heading;% - 2*kalman_state.X(5);
        Rate(i) = nav_params.angular_rate;% - 2*kalman_state.X(6);
        Velocity(i,1)=nav_params.velocity(1);
        Velocity(i,2)=nav_params.velocity(2);
        Betta(i,:) = nav_params.betta;
        Omega(i,:) = nav_params.omega;
        Gamma(i,:) = nav_params.gamma;
        vect(i,:) = kalman_state.X;
%         velc(i)= sqrt((nav_params.velocity(1) - kalman_state.X(3))^2 + (nav_params.velocity(2) - kalman_state.X(4))^2);
        
    end
%     close all
%     figure
%     plot(Y(1:limit),X(1:limit));
%     axis equal
%     grid on
%     title('Trajectory');
%     
%     figure;
%     plot(time(1:limit), Heading(1:limit))
%     title('Heading')
%     grid on
%     
%     figure;
%     plot(time(1:limit), Velocity(1:limit))
%     title('Velocity')
%     grid on
%     
%     figure;
%     plot(time(1:limit), Rate(1:limit))
%     title('Angular Rate')
%     grid on
end