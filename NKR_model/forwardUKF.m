function [state_prime] = forwardUKF(state, sensors)
    
    dt = sensors.dt;


    lsx = -0.5;
    lsy = 0.1;
    lf = 0.4;
    lr = 0.4;
    lw = 0.6;
    rw = 0.254/2;
    
    % State vector:
%     X - 1
%     Y - 2
%     dX - 3
%     dY - 4
%     Psi - 5
%     dPsi - 6
%     Gamma1 - 7
%     Gamma2 - 8
%     Omega1 - 9
%     Omega2 - 10
%     Omega3 - 11
%     Omega4 - 12
%     Betta1 - 13
%     Betta2 - 14
%     Betta3 - 15
%     Betta4 - 16
%     Kappa - 17

    U = [state.epos, state.epos, state.evel, state.evel, state.epsi, state.edpsi, ...
        state.egamma, state.egamma, state.ew, state.ew, state.ew, state.ew, ...
        state.eb, state.eb, state.eb, state.eb, state.ek]';
    Q = diag(U);
    
    %% Unscented transform
%     dim = 17;
%     n = 2 * dim + 1; % number of sigma-points
%     k = 15;
%     a = 0.25;
%     l = a * a * (dim + k) - dim;
%     b = 2; % for gaussians
%     Xs = zeros(dim, n);
%     Ws = zeros(2, n);
%     Xprime = zeros(dim, 1);
%     Pprime = zeros(dim, dim);
%     
%     % The first column repeats the state vector
%     Xs(:,1) = state.X; % mean
% %     fprintf("%7.6g \n", det((dim+l)*state.P));
%     Shift = chol((dim+l)*state.P);
%     
% %     if p > 0
% %         state_prime = state;
% %         fprintf("error occured\n")
% %         return;
% %     end
%     % For each parameter in X we need to calculate 18 additional
%     % sigma-points. This can be done by shifting the mean by values in the
%     % Shift matrix (that correlates with covariance matrix P)
%     for i = 1:dim
%         for j = 1:dim
%             Xs(i, j+1) = Xs(i,1) + Shift(i, j);
%             Xs(i, j+1 + dim) = Xs(i,1) - Shift(i, j);
%         end
%     end
%     % Now we need to compute weights for each sigma point in Xs.
%     % For each column in Xs we need mean_weight and covariance_weight
%     % The weights for the first column are the largest
%     Ws(1,1) = 1 / (dim + l);
%     Ws(2,1) = Ws(1,1) + (1 - a*a + b);
%     for i = 2:n
%         Ws(1,i) = 1 / (2*dim + 2*l);
%         Ws(2,i) = Ws(1,i);
%     end
%     % Normalize the weights
%     sumW1 = sum(Ws(1,:));
%     sumW2 = sum(Ws(2,:));
%     Ws(1,:) = Ws(1,:) ./ sumW1;
%     Ws(2,:) = Ws(2,:) ./ sumW2;
    [Xs, Ws] = unscented_transform(state.X, state.P);
    
    %% Prediction
    % Motion transform (apply system dynamic F)
    dim = length(state.X);
    n = 2 * length(state.X) + 1;
    
    % Apply prediction update to every column of Xs
    for j = 1:n
        Xs(1,j) = Xs(1,j) + Xs(3,j)*dt; % X
        Xs(2,j) = Xs(2,j) + Xs(4,j)*dt; % Y
        Xs(3,j) = rw / 4 * cos(Xs(5,j) + Xs(17,j)) * (Xs(9,j) + Xs(10,j) + Xs(11,j) + Xs(12,j)); % dX
        Xs(4,j) = rw / 4 * sin(Xs(5,j) + Xs(17,j)) * (Xs(9,j) + Xs(10,j) + Xs(11,j) + Xs(12,j));
        Xs(5,j) = Xs(5,j) + Xs(6,j)*dt; % Heading
        Xs(6,j) = rw / 4 * (Xs(9,j) + Xs(10,j) + Xs(11,j) + Xs(12,j)) * cos(Xs(17,j)) / (lf + lr) ...
            * (tan( 0.5*(Xs(7,j) + Xs(8,j) + Xs(13,j) + Xs(14,j)) ) - tan( 0.5*(Xs(15,j) + Xs(16,j)) )); % dHeading
        
        Xs(7,j) = Xs(7,j); % Gamma1
        Xs(8,j) = Xs(8,j); % Gamma2
%         Xs(7,j) = sensors.gamma - sensors.gamma^2 * lw / (lf + lr);
%         Xs(8,j) = sensors.gamma + sensors.gamma^2 * lw / (lf + lr);
        
        Xs(9,j) = Xs(9,j); % Omega1
        Xs(10,j) = Xs(10,j); % Omega2
        Xs(11,j) = Xs(11,j); % Omega3
        Xs(12,j) = Xs(12,j); % Omega4
        
        Xs(13,j) = Xs(13,j); % Betta1
        Xs(14,j) = Xs(14,j); % Betta2
        Xs(15,j) = Xs(15,j); % Betta3
        Xs(16,j) = Xs(16,j); % Betta4
        
        Xs(17,j) = atan( (lf * tan( 0.5*(Xs(15,j) + Xs(16,j)) ) + lr * tan( 0.5*(Xs(7,j) + Xs(8,j) + Xs(13,j) + Xs(14,j)) )) / (lf + lr) ); % Kappa
%         Xs(17,j) = atan2(Xs(4,j), Xs(3,j)) - Xs(5,j);
    end
    
    %% Reverse unscented transform
%     % Derive Xprime as weighted mean
%     for i = 1:dim
%         Xprime(i) = 0;
%         for j = 1:n
%             Xprime(i) = Xprime(i) + Xs(i,j)*Ws(1,j);
%         end
%     end
%     % Derive Pprime as weighted covariance
%     for j = 1:n
% %         disp(j)
%         a = Ws(2,j) * ((Xs(:,j) - Xprime) * (Xs(:,j) - Xprime)');
% %         disp(a)
%         Pprime = Pprime + a;
%     end
%     Pprime = Pprime + diag(U);
% 
% 
% %     P = Pprime;
% %     X = Xprime;
    [Xprime, Pprime] = reverse_unscented_transform(Xs, Ws);
    Pprime = Pprime + diag(U);
    Xprime(5) = wrapToPi(Xprime(5));
    
    %% Recalculate sigma points
%     [Xs, Ws] = unscented_transform(Xprime, Pprime);
    %% Unscented correction ODO
    Z = [sensors.omega(1);
        sensors.omega(2);
        sensors.omega(3);
        sensors.omega(4);
        sensors.gamma - sensors.gamma^2 * lw / (lf + lr);
        sensors.gamma + sensors.gamma^2 * lw / (lf + lr)];
     R = [sensors.eodow; sensors.eodow; sensors.eodow; sensors.eodow; sensors.eodog; sensors.eodog];
%     n = 2 * dim + 1; % number of sigma-points
    % Transform all the sigma points Xs through the measurement function
    Zs = zeros(6, n);
    for er = 1:n
        Zs(1,er) = Xs(9,er);
        Zs(2,er) = Xs(10,er);
        Zs(3,er) = Xs(11,er);
        Zs(4,er) = Xs(12,er);
        Zs(5,er) = Xs(7,er);
        Zs(6,er) = Xs(8,er);
    end
    % Derive Zhat - predicted measurement
    Zhat = zeros(6,1);
    for i = 1:6
        Zhat(i) = 0;
        for j = 1:n
            Zhat(i) = Zhat(i) + Zs(i,j)*Ws(1,j);
        end
    end
    
    % Matrix S
    S = zeros(6, 6);
    for j = 1:n
        S = S + Ws(2,j) * ((Zs(:,j) - Zhat) * (Zs(:,j) - Zhat)');
    end
    S = S + diag(R);
    
    % Matrix Sigma (x,z)
    Sig = zeros(dim, 6);
    for j = 1:n
        Sig = Sig + Ws(2,j) * ((Xs(:,j) - Xprime) * (Zs(:,j) - Zhat)');
    end
    % Kalman gain
    K = Sig * inv(S);
    Xprime = Xprime + K*(Z - Zhat);
    Pprime = Pprime - K*S*K';
    
    %% Linear Correction ODO
%     Z = [sensors.omega(1);
%         sensors.omega(2);
%         sensors.omega(3);
%         sensors.omega(4);
%         sensors.gamma - sensors.gamma^2 * lw / (lf + lr);
%         sensors.gamma + sensors.gamma^2 * lw / (lf + lr)];
%     
%     H = zeros(6, 17);
%     H(1:4, 9:12) = eye(4);
%     H(5,7) = 1;
%     H(6,8) = 1;
%     R = [sensors.eodow; sensors.eodow; sensors.eodow; sensors.eodow; sensors.eodog; sensors.eodog];
%     I = eye(17);
%     Y = Z - H * Xprime;
%     S = H * Pprime * H' + R;
%     K = Pprime * H' * inv(S);
%     Xprime = Xprime + K * Y;
%     Pprime = (I - K * H) * Pprime;
    
    %% Kalamn Correction GYRO
    if (sensors.gyro_update == 1)
        Z = [sensors.gyro];
        H = zeros(1,17);
        H(6) = 1;
        R = [sensors.egyro];
        I = eye(17);
        Y = Z - H * Xprime;
        S = H * Pprime * H' + R;
        K = Pprime * H' * inv(S);
        Xprime = Xprime + K * Y;
        Pprime = (I - K * H) * Pprime;
    end
    %% Recalculate sigma points
    [Xs, Ws] = unscented_transform(Xprime, Pprime);
    %% Unscented correction GPS
    if (sensors.gps_update == 1)
        Z = [sensors.gps_x;
                sensors.gps_y;
                sensors.gps_dx;
                sensors.gps_dy];
        R = [sensors.egps_pos; sensors.egps_pos; sensors.egps_vel; sensors.egps_vel];
        n = 2 * dim + 1; % number of sigma-points
        % Transform all the sigma points Xs through the measurement function
        Zs = zeros(4, n);
        for er = 1:n
            Zs(1,er) = Xs(1,er) + lsx * cos(Xs(5,er)) - lsy * sin(Xs(5, er));
            Zs(2,er) = Xs(2,er) + lsx * sin(Xs(5,er)) + lsy * cos(Xs(5,er));
            Zs(3,er) = Xs(3,er) + Xs(6,er) * (lsx * cos(Xs(5,er)) - lsy * sin(Xs(5,er)));
            Zs(4,er) = Xs(4,er) + Xs(6,er) * (lsx * sin(Xs(5,er)) + lsy * cos(Xs(5,er)));
        end
        % Derive Zhat - predicted measurement
        Zhat = zeros(4,1);
        for i = 1:4
            Zhat(i) = 0;
            for j = 1:n
                Zhat(i) = Zhat(i) + Zs(i,j)*Ws(1,j);
            end
        end

        % Matrix S
        S = zeros(4, 4);
        for j = 1:n
            S = S + Ws(2,j) * ((Zs(:,j) - Zhat) * (Zs(:,j) - Zhat)');
        end
        S = S + diag(R);

        % Matrix Sigma (x,z)
        Sig = zeros(dim, 4);
        for j = 1:n
            Sig = Sig + Ws(2,j) * ((Xs(:,j) - Xprime) * (Zs(:,j) - Zhat)');
        end
        % Kalman gain
        K = Sig * inv(S);
        Xprime = Xprime + K*(Z - Zhat);
        Pprime = Pprime - K*S*K';
    end
    %% Linear Correction GPS
%     if (sensors.gps_update == 1)
%         Z = [sensors.gps_x - lsx * cos(Xprime(5)) + lsy * sin(Xprime(5));
%             sensors.gps_y - lsx * sin(Xprime(5)) - lsy * cos(Xprime(5));
%             sensors.gps_dx;
%             sensors.gps_dy];
%         R = [sensors.egps_pos; sensors.egps_pos; sensors.egps_vel; sensors.egps_vel];
%         H = zeros(4,17);
%         H(1:2,1:2) = eye(2);
%         H(3:4,3:4) = eye(2);
%         H(3,6) = lsx * cos(Xprime(5)) - lsy * sin(Xprime(5));
%         H(4,6) = lsx * sin(Xprime(5)) + lsy * cos(Xprime(5));
%         I = eye(17);
%         Y = Z - H * Xprime;
%         S = H * Pprime * H' + R;
%         K = Pprime * H' * inv(S);
%         Xprime = Xprime + K * Y;
%         Pprime = (I - K * H) * Pprime;
%     end
    %%
    state_prime = state;
    state_prime.P = Pprime;
    state_prime.X = Xprime;
end

