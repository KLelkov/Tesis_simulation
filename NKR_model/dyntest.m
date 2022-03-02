simTime = 20;
dt = 0.01;
nSim = simTime / dt;
xg = zeros(nSim, 1);
yg = zeros(nSim, 1);
psi = zeros(nSim, 1);
vxg = zeros(nSim, 1);
vyg = zeros(nSim, 1);
vxb = zeros(nSim, 1);
vyb = zeros(nSim, 1);
dpsi = zeros(nSim, 1);
ddpsi = zeros(nSim, 1);
axb = zeros(nSim, 1);
ayb = zeros(nSim, 1);
w = zeros(nSim, 4);
Mw = zeros(nSim, 4);
ew_int = zeros(1, 4);
Mf = zeros(nSim, 4);
dw = zeros(nSim,4);
Ms = zeros(nSim, 4);
Ft = zeros(nSim, 4);
gamma = zeros(nSim, 2);

w1 = 0;
w2 = 0;
w3 = 0;
w4 = 0;
gamma1 = 0;
gamma2 = 0;
m = 50;
g = 9.8;
a = 0.007;
k = 0.1;
Jrot = 0.015;
Jzz = 2.7;
rw = 0.254/2;
lw = 0.6;
lf = 0.4;
wcmd = [14 14 14 14];

e1 = 0;
for i = 2:nSim
    % Wheels rotation control
    for j = 1:4
        Mw(i,j) = 0.25 * (wcmd(j) - w(i-1,j)) + 0.005 * ew_int(j);
        ew_int(j) = ew_int(j) + (wcmd(j) - w(i-1,j));
        Mw(i,j) = bound(Mw(i,j), 0, 3);
        Mf(i,j) = bound(m*g*a/4, 0, Mw(i,j));
        w(i,j) = w(i-1,j) + dw(i-1,j) * dt;
        % Slippage doesnt affect wheel's rotation speed
        dw(i,j) = (Mw(i,j) - Mf(i,j)) / Jrot;
    end
    
    % Slippage moments cannot exceed total grip moment relative to the
    % surface
    for j = 1:4
        Ms(i,j) = bound(Ms(i,j), 0, Mw(i,j) - Mf(i,j));
        % Total force on each wheel
        Ft(i,j) = (Mw(i,j) - Mf(i,j) - Ms(i,j)) / rw;
    end
    
    xg(i) = xg(i-1) + vxg(i-1) * dt;
    yg(i) = yg(i-1) + vyg(i-1) * dt;
    psi(i) = psi(i-1) + dpsi(i-1) * dt;
    vxg(i) = vxg(i-1) + axb(i-1) * cos(psi(i-1)) * dt - ayb(i-1) * sin(psi(i-1)) * dt;
    vyg(i) = vyg(i-1) + axb(i-1) * sin(psi(i-1)) * dt + ayb(i-1) * cos(psi(i-1)) * dt;
    vxb(i) = vxb(i-1) + axb(i-1) * dt;
    vyb(i) = vyb(i-1) + ayb(i-1) * dt;
    dpsi(i) = dpsi(i-1) + ddpsi(i-1) * dt;
    axb(i) = Ft(i,1) * cos(gamma(i,1)) + Ft(i,2) * cos(gamma(i,2)) + Ft(i,3) + Ft(i,4) + 2 * m * vyb(i) * dpsi(i);
    ayb(i) = Ft(i,1) * sin(gamma(i,1)) + Ft(i,2) * sin(gamma(i,2)) - 2 * m * vxb(i) * dpsi(i);
    axb(i) = axb(i) / m;
    ayb(i) = ayb(i) / m;
    ddpsi(i) = 0.5 * lw * (Ft(i,1) * cos(gamma(i,1)) - Ft(i,2) * cos(gamma(i,2)) + Ft(i,3) - Ft(i,4)) + lf * (Ft(i,1) * sin(gamma(i,1)) + Ft(i,2) * sin(gamma(i,2)));
    ddpsi(i) = ddpsi(i) / Jzz;

    
end
close all
plot(w)
figure
plot(yg, xg)
figure
plot(vxb)
figure
plot(Mw)