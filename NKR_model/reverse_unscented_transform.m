function [Xprime, Pprime] = reverse_unscented_transform(Xs, Ws)
%% Unscented transform
    dim = (length(Ws) - 1) / 2;
    n = 2 * dim + 1; % number of sigma-points
    k = 15;
    a = 0.25;
    l = a * a * (dim + k) - dim;
    b = 2; % for gaussians
    Xprime = zeros(dim, 1);
    Pprime = zeros(dim, dim);
    for i = 1:dim
        Xprime(i) = 0;
        for j = 1:n
            Xprime(i) = Xprime(i) + Xs(i,j) * Ws(1,j);
        end
    end
    % Derive Pprime as weighted covariance
    for j = 1:n
        a = Ws(2,j) * ((Xs(:,j) - Xprime) * (Xs(:,j) - Xprime)');
        Pprime = Pprime + a;
    end
%     Pprime = Pprime + diag(U);

%     Xprime(5) = wrapToPi(Xprime(5));
end

