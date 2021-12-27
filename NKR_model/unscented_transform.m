function [Xs, Ws] = unscented_transform(X,P)
%% Unscented transform
    dim = length(X);
    n = 2 * dim + 1; % number of sigma-points
    k = 15;
    a = 0.25;
    l = a * a * (dim + k) - dim;
    b = 2; % for gaussians
    Xs = zeros(dim, n);
    Ws = zeros(2, n);
    
    % The first column repeats the state vector
    Xs(:,1) = X; % mean
    Shift = chol((dim+l) * P);
    
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
end

