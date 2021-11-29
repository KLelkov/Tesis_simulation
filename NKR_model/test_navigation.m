function [X, Y, Heading] = test_navigation(time, gamma1, gamma2, Gyro, Lat, Lon, Vn, Ve, w1, w2, w3, w4)
    len = length(time);
    referenceNeeded = true;
    notMovedYet = true;
    validCoords = false;
    sensors.dt = 0.01;
    referenceLat = 0;
    referenceLon = 0;
    X = zeros(len, 1);
    Y = zeros(len, 1);
    Heading = zeros(len, 1);
    
    
    % init sensors struct
    sensors.gps_error_pos = 3e0;
    sensors.gps_error_vel = 6e-1;
    sensors.gyro_error = 4e-1;
    sensors.odo_error = 1e0;%8e-1;
    
    %init Kalman state struct
    kalman_state.X = zeros(9,1);
    kalman_state.X(3) = 0; % set initial heading
    kalman_state.P = diag([100 100 10 10 10 10 10 10 10]);
    kalman_state.pos = 2e-5;
    kalman_state.h = 1e-4;
    kalman_state.v = 3e-3;
    kalman_state.dh = 8e-4;
    kalman_state.odo = 1e-2;%8e-3;
    
    for i = 1:500
        if i > 1
            sensors.dt = (time(i) - time(i-1)) / 1000;
        end
        if w1(i) > 0.2
            sensors.w1 = w1(i);
        else
            sensors.w1= 0;
        end
        if w2(i) > 0.2
            sensors.w2 = w2(i);
        else
            sensors.w2= 0;
        end
        if w3(i) > 0.2
            sensors.w3 = w3(i);
        else
            sensors.w3= 0;
        end
        if w4(i) > 0.2
            sensors.w4 = w4(i);
        else
            sensors.w4= 0;
        end
        sensors.gammaf = gamma1(i);
        sensors.gammar = gamma2(i);
        if sensors.w1 ~= 0 || sensors.w2 ~= 0 || sensors.w3~= 0 || sensors.w4 ~= 0
            notMovedYet = false;
        end
        sensors.gyro = Gyro(i);
        validCoords = false;
        if Lat(i) > 53 && Lat(i) < 58
            if Lon(i) > 35 && Lon(i) < 40
                validCoords = true;
            end
        end
        if validCoords
            referenceNeeded = false;
            if referenceLat == 0 || referenceLon == 0
                referenceNeeded = true;
            end
            if referenceNeeded
                shiftLat = 0; shiftLon = 0;
                if i > 1
                    shiftLat = X(i-1) * pi / 111111 / 180.0;
                    shiftLon = Y(i-1) * pi / (111111 * cosd(Lat(i))) / 180.0;
                end
                referenceLat = Lat(i) * pi / 180 - shiftLat;
                referenceLon = Lon(i) * pi / 180 - shiftLon;
            end
            if notMovedYet
                referenceLat = Lat(i)  * pi / 180;
                referenceLon = Lon(i)  * pi / 180;
            end
            sensors.gps_x = (Lat(i)*pi/180 - referenceLat) * 111111;
            sensors.gps_y = (Lon(i)*pi/180 - referenceLon) * (111111 * cosd(Lat(i)));
            sensors.gps_dx = Vn(i);
            sensors.gps_dy = Ve(i);
        end
        if rem(i, 100) == 0
            update.gps = 1;
        else
            update.gps = 0;
        end
        update.gyro = 1;
        
        kalman_state = ukfNav(kalman_state, sensors, update);
        X(i) = kalman_state.X(1);
        Y(i) = kalman_state.X(2);
        Heading(i) = kalman_state.X(3);
        
    end
    figure
    plot(Y,X);
    axis equal
    grid on
    figure;
    plot(Heading)
    grid on
end