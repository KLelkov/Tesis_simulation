function [controls, control_params] = uav_locomotion_control(state, controls, target, control_params)
    W = controls;
    
    g = 9.81;
    ly = 0.3;
    lx = 0.25;
    r = 0.1;
    lxp = 0.18;
    lyp2 = 0.23;
    lyp1 = 0.12;
    lxy = sqrt(lx^2 + ly^2);
    ro = 1.2;
    k = 1.4e-5;
    kr = 0.08;
    
    
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
    
    % Rotation matrix
    Rgb = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch);
        cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), cos(pitch)*sin(roll);
        cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), cos(pitch)*cos(roll)];
    
    % Transition from body frame to geo frame
    dVg = Rgb' * [ddxb; ddyb; ddzb];
    dyaw = (wzb * cos(roll) + wyb * sin(roll)) / cos(pitch);
    dpitch = wyb * cos(roll) - wzb * sin(roll);
    droll = wxb + dyaw * sin(pitch);
    
    % Normal frame. X is the forward motion of UAV, Y is the lateral motion
    dVxn = dVg(1) * cos(yaw) + dVg(2) * sin(yaw);
    dVyn = -dVg(1) * sin(yaw) + dVg(2) * cos(yaw);
    Vxn = dxg * cos(yaw) + dyg * sin(yaw);
    Vyn = -dxg * sin(yaw) + dyg * cos(yaw);

    
    % Propulsion moments on motors
    T = zeros(1,6);
    for j = 1:6
        vzi = k * W(j)^2;
        T(j) = 2 * ro * pi * r^2 * vzi * (vzi - dzb);
    end
    
    %% Vertical velocity
    % Vertial velocity difference -> vetical acceleration control
    vze = target.dvz - dzg;
    vz_control = 17.3 * vze - 4.25 * dVg(3);
    
    %% Forward velocity
    % Forward velocity difference -> forward accleration (pitch) control
    vxe = target.forward_vel - Vxn;
    vx_control = -1.06 * vxe + 0.29 * dVxn;
    % Pitch difference -> dPitch control
    vx_control = bound(vx_control, -20*pi/180.0, 20*pi/180.0);
    pe1 = vx_control - pitch;
    dp_control = 1.75 * pe1 - 0.31 * dpitch;
    % dPitch difference -> ddPitch control (Myb)
    pe = dp_control - dpitch;
    pitch_control = 0.37 * pe - 0.02*dwyb;
    
    %% Lateral velocity
    % Lateral velocity difference -> lateral acceleration (roll) control
    vye = target.dvy - Vyn;
    vy_control = 0.96 * vye - 0.27 * dVyn;
    % Roll difference -> dRoll control
    vy_control = bound(vy_control, -20*pi/180.0, 20*pi/180.0);
    re1 = vy_control - roll;
    dr_control = 1.78 * re1 - 0.14 * droll;
    % dRoll difference -> ddRoll control (Mxb)
    re = dr_control - droll;
    roll_control = 0.47 * re - 0.02 * dwxb;
    
    %% Yaw channel
    % Yaw difference -> dYaw control
    ye1 = target.yaw - yaw;
    ye1 = pi2pi(ye1);
    dy_control = 2.78 * ye1 - 0.21 * dyaw;
    % dYaw difference -> Mz control
    ye = dy_control - dyaw;
    yaw_control = 0.63 * ye - 0.05 * dwzb;
    

    %% Convert input signals to control values
    % Convert control moments into separate motors lift force change
    Mx = roll_control;
    My = pitch_control;
    Mz = yaw_control;
    R = -vz_control;
%     T1 = R/6;
%     T2 = (3*My*kr*lyp1 - 3*Mx*kr*lxp + 3*My*kr*lyp2 - 3*Mz*lxp*lyp1 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
%     T3 = -(3*Mx*kr*lxp + 3*My*kr*lyp1 + 3*My*kr*lyp2 - 3*Mz*lxp*lyp2 - R*kr*lxp*lyp1 - R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
%     T4 = (3*Mx*kr*lxp - 3*My*kr*lyp1 - 3*My*kr*lyp2 - 3*Mz*lxp*lyp2 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
%     T5 = (3*Mx*kr*lxp + 3*My*kr*lyp1 + 3*My*kr*lyp2 + 3*Mz*lxp*lyp1 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
%     T6 = R/6;
    T1 = (3*My*kr*lyp1 - 3*Mx*kr*lxp + 3*Mz*lxp*lyp1 + 2*R*kr*lxp*lyp1)/(12*kr*lxp*lyp1);
    T3 = -(3*Mx*kr*lxp + 3*My*kr*lyp1 + 3*Mz*lxp*lyp1 - 2*R*kr*lxp*lyp1)/(12*kr*lxp*lyp1);
    T4 = (3*Mx*kr*lxp - 3*My*kr*lyp1 + 3*Mz*lxp*lyp1 + 2*R*kr*lxp*lyp1)/(12*kr*lxp*lyp1);
    T6 = (3*Mx*kr*lxp + 3*My*kr*lyp1 - 3*Mz*lxp*lyp1 + 2*R*kr*lxp*lyp1)/(12*kr*lxp*lyp1);
    T2 = mean([T1, T3, T4, T6]);
    T5 = mean([T1, T3, T4, T6]);
 
    
    dw1 = sign(T1)*sqrt(abs(T1) / 2 / ro / pi / r^2);
    dw2 = sign(T2)*sqrt(abs(T2) / 2 / ro / pi / r^2);
    dw3 = sign(T3)*sqrt(abs(T3) / 2 / ro / pi / r^2);
    dw4 = sign(T4)*sqrt(abs(T4) / 2 / ro / pi / r^2);
    dw5 = sign(T5)*sqrt(abs(T5) / 2 / ro / pi / r^2);
    dw6 = sign(T6)*sqrt(abs(T6) / 2 / ro / pi / r^2);
    
    w1 = controls(1) + dw1;
    w2 = controls(2) + dw2;
    w3 = controls(3) + dw3;
    w4 = controls(4) + dw4;
    w5 = controls(5) + dw5;
    w6 = controls(6) + dw6;
    
    controls(1) = bound(w1, 0, 1500);
    controls(2) = bound(w2, 0, 1500);
    controls(3) = bound(w3, 0, 1500);
    controls(4) = bound(w4, 0, 1500);
    controls(5) = bound(w5, 0, 1500);
    controls(6) = bound(w6, 0, 1500);

    
end

