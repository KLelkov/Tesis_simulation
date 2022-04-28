function state_prime = uav_sim_step(state, controls)
    state_prime = state;
    W = controls;
    dt = 0.01;
    m = 6;
    g = 9.81;
    ly = 0.3;
    lz = 0.1;
    lx = 0.25;
    r = 0.1;
    lxp = 0.18;
    lyp2 = 0.23;
    lyp1 = 0.12;
    lxy = sqrt(lx^2 + ly^2);
    ro = 1.2;
    Sxy = 0.4;
    Sxz = 0.15;
    Sx = lz*ly;
    Sy = lz*lx;
    Sz = lx*ly;
    k = 1.4e-5;
    cx = -0.08;
    cy = -0.08;
    cz = -0.12;
    mx = -0.05;
    my = -0.05;
    mz = -0.02;
    kr = 0.08;
    rr = 0.035 / 2;
    Jr = 0.5 * 0.2 * rr^2;
    Jxx = 0.25 * m *r^2 + 1 / 12 * m *lz^2;
    Jyy = 0.25 * m *r^2 + 1 / 12 * m *lz^2;
    Jzz = 0.5 * m *r^2;
    Jyz = 0.002;
    Jxz = 0.002;
    Jxy = 0.005;
    
    cx = 0;
    cy = 0;
    cz = 0;
    mx = 0;
    my = 0;
    mz = 0;
    
    xg = state.xg;
    yg = state.yg;
    zg = state.zg;
    dxb = state.dxb;
    dyb = state.dyb;
    dzb = state.dzb;
    ddxb = state.ddxb;
    ddyb = state.ddyb;
    ddzb = state.ddzb;
    wxb = state.wxb;
    wyb = state.wyb;
    wzb = state.wzb;
    yaw = state.yaw;
    pitch = state.pitch;
    roll = state.roll;
    dwxb = state.dwxb;
    dwyb = state.dwyb;
    dwzb = state.dwzb;
    dxg = state.dxg;
    dyg = state.dyg;
    dzg = state.dzg;
    
    
    % Propulsion moments on motors
    T = zeros(1,6);
    for j = 1:6
        vzi = k * W(j)^2;
        T(j) = 2 * ro * pi * r^2 * vzi * (vzi);% - dzb);
    end
    
    
    
    % Full velocity in body frame
    Vb = sqrt(dxb^2 + dyb^2 + dzb^2);
    
    % Forces
    Fbg = [-m*g*sin(pitch);
        m*g*cos(pitch)*sin(roll);
        m*g*cos(pitch)*cos(roll)];
    Fba = [0.5*cx*ro*Vb^2*Sx;
        0.5*cy*ro*Vb^2*Sy;
        0.5*cz*ro*Vb^2*Sz];
    Fbp = [0;
        0;
        -sum(T)];
    % Moments
    Mbp = [lyp1*(T(6)+T(4)-T(3)-T(1)) + lyp2*(T(5)-T(2));
        lxp*(T(1) - T(3) - T(4) + T(6));
        kr*(T(1)-T(2)-T(3)+T(4)+T(5)-T(6))];
    Mbgp = [Jr*wyb*(W(1) - W(2) + W(3) - W(4) + W(5) - W(6));
        -Jr*wxb*(W(1) - W(2) - W(3) + W(4) + W(5) - W(6));
        0];
    Mba = [0.5*mx*ro*Vb^2*Sxy*ly;
        0.5*my*ro*Vb^2*Sxy*lx;
        0.5*mz*ro*Vb^2*Sxz*lxy];
    
    % Coordinate update
    xg = xg + dxg * dt;
    yg = yg + dyg * dt;
    zg = zg + dzg * dt;
    
    dyaw = (wzb * cos(roll) + wyb * sin(roll)) / cos(pitch);
    dpitch = wyb * cos(roll) - wzb * sin(roll);
    droll = wxb + dyaw * sin(pitch);
    
    yaw = yaw + dyaw * dt;
    pitch = pitch + dpitch * dt;
    roll = roll + droll *dt;
    
    % Dynamic equations
    dxb = dxb + ddxb * dt;
    dyb = dyb + ddyb * dt;
    dzb = dzb + ddzb * dt;
    wxb = wxb + dwxb * dt;
    wyb = wyb + dwyb * dt;
    wzb = wzb + dwzb * dt;
    ddxb = (Fbg(1) + Fba(1) + Fbp(1) - 2 * m * (dyb * wzb - wyb * dzb)) / m;
    ddyb = (Fbg(2) + Fba(2) + Fbp(2) - 2 * m * (dzb * wxb - wzb * dxb)) / m;
    ddzb = (Fbg(3) + Fba(3) + Fbp(3) - 2 * m * (dxb * wyb - wxb * dyb)) / m;
    dwxb = (Mba(1) + Mbgp(1) + Mbp(1) - wyb * wzb * Jyz) / Jxx;
    dwyb = (Mba(2) + Mbgp(2) + Mbp(2) - wxb * wzb * Jxz) / Jyy;
    dwzb = (Mba(3) + Mbgp(3) + Mbp(3) - wxb * wyb * Jxy) / Jzz;
    
    % Transition from body frame to geo frame
    % Rotation matrix from geo to body
    Rgb = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch);
        cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), cos(pitch)*sin(roll);
        cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), cos(pitch)*cos(roll)];
    Vg = Rgb' * [dxb; dyb; dzb];
    dVg = Rgb' * [ddxb; ddyb; ddzb];
    
    % Yes, this seems weird to duplicate this, but the old values are
    % actually outdated by now 0_o
    % This is related to the issue of simulating inertia of parameters
    % change:
    % Acceleration will transition to velocity one sim step later
    % and velocity will transition to coordinates another sim step later
    %
    % TODO: there's probably a smarter way to do this.
    dyaw = (wzb * cos(roll) + wyb * sin(roll)) / cos(pitch);
    dpitch = wyb * cos(roll) - wzb * sin(roll);
    droll = wxb + dyaw * sin(pitch);
    
    
    state_prime.xg = xg;
    state_prime.yg = yg;
    state_prime.zg = zg;
    state_prime.dxb = dxb;
    state_prime.dyb = dyb;
    state_prime.dzb = dzb;
    state_prime.ddxb = ddxb;
    state_prime.ddyb = ddyb;
    state_prime.ddzb = ddzb;
    state_prime.wxb = wxb;
    state_prime.wyb = wyb;
    state_prime.wzb = wzb;
    state_prime.yaw = yaw;
    state_prime.pitch = pitch;
    state_prime.roll = roll;
    state_prime.dwxb = dwxb;
    state_prime.dwyb = dwyb;
    state_prime.dwzb = dwzb;
    state_prime.dxg = Vg(1);
    state_prime.dyg = Vg(2);
    state_prime.dzg = Vg(3);
    state_prime.ddxg = dVg(1);
    state_prime.ddyg = dVg(2);
    state_prime.ddzg = dVg(3);
    
    state_prime.dyaw = dyaw;
    state_prime.dpitch = dpitch;
    state_prime.droll = droll;
end