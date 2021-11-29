function state_prime = slipNav(state, sensors, update)
    
    dt = 0.01;
    tangf = tan(sensors.gammaf);
    tangr = tan(sensors.gammar);
    lsx = 0.15;
    lsy = 0.05;
    lf = 0.4;
    lr = 0.4;
    lw = 0.3;
    rw = 0.1;
    U = [state.pos state.pos state.h state.v state.dh state.odo state.odo state.odo state.odo state.slip];
    
    
    
    %% Unscented transform
    dim = 10;
    n = 2 * dim + 1; % number of sigma-points
    k = 15;
    a = 0.25;
    l = a * a * (dim + k) - dim;
    b = 2; % for gaussians
    Xs = zeros(dim, n);
    Ws = zeros(2, n);
    Xprime = zeros(dim, 1);
    Pprime = zeros(dim, dim);
    
    % The first column repeats the state vector
    Xs(:,1) = state.X; % mean
    Shift = chol((dim+l)*state.P);
    % For each parameter in X we need to calculate 18 additional
    % sigma-points. This can be done by shifting the mean by values in the
    % Shift matrix (that correlates with covariance matrix P)
    for i = 1:dim
        for j = 1:dim
            Xs(i, j+1) = Xs(i,1) + Shift(i, j);
            Xs(i, j+1 + dim) = Xs(i,1) - Shift(i, j);
        end
    end
    % Now we need to compute weights for each sigma point in Xs.
    % For each column in Xs we need mean_weight and covariance_weight
    % The weights for the first column are the largest
    Ws(1,1) = 1 / (dim + l);
    Ws(2,1) = Ws(1,1) + (1 - a*a + b);
    for i = 2:n
        Ws(1,i) = 1 / (2*dim + 2*l);
        Ws(2,i) = Ws(1,i);
    end
    % Normalize the weights
    sumW1 = sum(Ws(1,:));
    sumW2 = sum(Ws(2,:));
    Ws(1,:) = Ws(1,:) ./ sumW1;
    Ws(2,:) = Ws(2,:) ./ sumW2;
    
    %% Prediction
    % Motion transform (apply system dynamic F)


    % ������� ������� ������������� (��� ������ ����. - ��� ������ ���
    % ��������)
    
    
    % Apply prediction update to every column of Xs
    for j = 1:n
        Xs(1,j) = Xs(1,j) + Xs(4,j)*cos(Xs(3,j)+Xs(10,j))*dt; % X
        Xs(2,j) = Xs(2,j) + Xs(4,j)*sin(Xs(3,j)+Xs(10,j))*dt; % Y
        Xs(3,j) = Xs(3,j) + Xs(5,j)*dt; % Heading
        Xs(4,j) = rw / 4 * ( Xs(6,j) + Xs(7,j) + Xs(8,j) + Xs(9,j) ); % V
        Xs(5,j) = 1.0 * (tangf-tangr)/(lf+lr) * Xs(4,j) * cos(Xs(10,j)); % dHeading
%         Xs(5,j) = Xs(5,j) + 0.25 * rw / (2 * lw) * (Xs(6,j) - Xs(7,j) + Xs(8,j) - Xs(9,j));
        Xs(6,j) = Xs(6,j); % w1
        Xs(7,j) = Xs(7,j); % w2
        Xs(8,j) = Xs(8,j); % w3
        Xs(9,j) = Xs(9,j); % w4
        Xs(10,j) = Xs(10,j);
        
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
        Pprime = Pprime + Ws(2,j) * ((Xs(:,j) - Xprime) * (Xs(:,j) - Xprime)');
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
%         % ��������� ������ �������� (���������� ������)
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
%         % ��������� ������ �������� (���������� ������)
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
%         % ��������� ������ �������� (���������� ������)
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
%         % ��������� ������ �������� (���������� ������)
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
        H = [ 1, 0, 0,            0,     0, 0, 0, 0, 0, 0; % Xs
              0, 1, 0,            0,     0, 0, 0, 0, 0, 0; % Ys
              0, 0, 0, cos(heading+X(10)), xproj, 0, 0, 0 ,0, 0; % dXs
              0, 0, 0, sin(heading+X(10)), yproj, 0, 0, 0, 0, 0; % dYs
              0, 0, 0,            0,     1, 0, 0, 0, 0, 0; % dHeading_gyro
              0, 0, 0,            0,     0, 1, 0, 0, 0, 0; % odo1
              0, 0, 0,            0,     0, 0, 1, 0, 0, 0; % odo2
              0, 0, 0,            0,     0, 0, 0, 1, 0, 0; % odo3
              0, 0, 0,            0,     0, 0, 0, 0, 1, 0;% odo4
              0, 0, 0,            0,     0, 0, 0, 0, 0, 1];% odo4

        Z = [sensors.gps_x - xproj;
            sensors.gps_y - yproj;
            sensors.gps_dx;
            sensors.gps_dy;
            sensors.gyro;
            sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4;
            0];
        if abs(X(3)) > 0.01
            Z(10) = cot(X(3)) - sensors.gps_dx / (X(4) * sin(X(3)));
        else
            Z(10) = sensors.gps_dy / (X(4) * cos(X(3))) - tan(X(3));
        end
        % ��������� ������ �������� (���������� ������)
        R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error 1e0]);
    elseif update.gyro == 1
        %     X  Y     H.             V    dH. w1 w2 w3 w4
        H = [ 0, 0,     0,            0,     1, 0, 0, 0, 0, 0; % dHeading_gyro
              0, 0,     0,            0,     0, 1, 0, 0, 0, 0; % odo1
              0, 0,     0,            0,     0, 0, 1, 0, 0, 0; % odo2
              0, 0,     0,            0,     0, 0, 0, 1, 0, 0; % odo3
              0, 0,     0,            0,     0, 0, 0, 0, 1, 0];% odo4

        Z = [sensors.gyro;
            sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % ��������� ������ �������� (���������� ������)
        R = diag([sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    elseif update.gps == 1 % never happens (gps always comes with gyro)
        %     X  Y  H.            V     dH. w1 w2 w3 w4
        H = [ 1, 0, 0,            0,     0, 0, 0, 0, 0, 0; % Xs
              0, 1, 0,            0,     0, 0, 0, 0, 0, 0; % Ys
              0, 0, 0, cos(heading+X(10)), xproj, 0, 0, 0 ,0, 0; % dXs
              0, 0, 0, sin(heading+X(10)), yproj, 0, 0, 0, 0, 0; % dYs
              0, 0, 0,            0,     0, 1, 0, 0, 0, 0; % odo1
              0, 0, 0,            0,     0, 0, 1, 0, 0, 0; % odo2
              0, 0, 0,            0,     0, 0, 0, 1, 0, 0; % odo3
              0, 0, 0,            0,     0, 0, 0, 0, 1, 0];% odo4

        Z = [sensors.gps_x - xproj;
            sensors.gps_y - yproj;
            sensors.gps_dx;
            sensors.gps_dy;
            sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % ��������� ������ �������� (���������� ������)
        R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    else % only odometry is available (may happen if gyro measurements dont come at every simulation step)
        %     X  Y     H.             V    dH. w1 w2 w3 w4
        H = [ 0, 0,     0,            0,     0, 1, 0, 0, 0, 0; % odo1
              0, 0,     0,            0,     0, 0, 1, 0, 0, 0; % odo2
              0, 0,     0,            0,     0, 0, 0, 1, 0, 0; % odo3
              0, 0,     0,            0,     0, 0, 0, 0, 1, 0];% odo4

        Z = [sensors.w1;
            sensors.w2;
            sensors.w3;
            sensors.w4];
        % ��������� ������ �������� (���������� ������)
        R = diag([sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);
    end
    
    
    y = Z - H*X; % �������
    S = H * P * H' + R; 
    K = P * H' * inv(S);
    Xprime = X + K*y; % X'
    Pprime = (eye(10) - K*H)*P; % P'
    
    state_prime.P = Pprime;
    state_prime.X = Xprime;
    
end

