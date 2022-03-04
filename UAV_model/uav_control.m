function [controls] = uav_control(state, controls, target)
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
    
    % Propulsion moments on motors
    T = zeros(1,6);
    for j = 1:6
        vzi = k * W(j)^2;
        T(j) = 2 * ro * pi * r^2 * vzi * (vzi - dzb);
    end
    
    % Vertical velocity
    vze = target.dvz - dzg;
    vz_control = 9.3 * vze - 4.25 * dVg(3);
    
    % Forward velocity
    vxe = target.dvx - dxg;
    vx_control = 0.6 * vxe + 0.2 * dVg(1);
    
    % Lateral velocity
    vye = target.dvy - dyg;
    vy_control = 0.6 * vye - 0.2 * dVg(2);
    
    % Pitch
    pe = target.dpitch - dpitch;
    pitch_control = 0.0003 * pe - 0.0001 * dwyb
    
    % Roll
    re = vy_control - roll;
    roll_control = 0.8 * re + 0.1 * wxb;
    
    % Yaw
    ye = target.dyaw - dyaw;
    yaw_control = 0.03 * ye + 0.002 * dwzb;
    yaw_control  = 0.01;
    
    % Convert control moments into separate motors lift force change
    Mx = 0;%roll_control;
    My = pitch_control;
    Mz = 0;
    R = 0;%-vz_control;
%     T1 = R/2;
%     T2 = -(Mx*kr*lxp - My*kr*lyp1 - My*kr*lyp2 + Mz*lxp*lyp1 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(2*kr*lxp*(lyp1 + lyp2));
%     T3 = -(Mx*kr*lxp + My*kr*lyp1 + My*kr*lyp2 - Mz*lxp*lyp2 - R*kr*lxp*lyp1 - R*kr*lxp*lyp2)/(2*kr*lxp*(lyp1 + lyp2));
%     T4 = (Mx*kr*lxp - My*kr*lyp1 - My*kr*lyp2 - Mz*lxp*lyp2 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(2*kr*lxp*(lyp1 + lyp2));
%     T5 = (Mx*kr*lxp + My*kr*lyp1 + My*kr*lyp2 + Mz*lxp*lyp1 - R*kr*lxp*lyp1 - R*kr*lxp*lyp2)/(2*kr*lxp*(lyp1 + lyp2));
%     T6 = R/2;
    T1 = R/6;
    T2 = (3*My*kr*lyp1 - 3*Mx*kr*lxp + 3*My*kr*lyp2 - 3*Mz*lxp*lyp1 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
    T3 = -(3*Mx*kr*lxp + 3*My*kr*lyp1 + 3*My*kr*lyp2 - 3*Mz*lxp*lyp2 - R*kr*lxp*lyp1 - R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
    T4 = (3*Mx*kr*lxp - 3*My*kr*lyp1 - 3*My*kr*lyp2 - 3*Mz*lxp*lyp2 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
    T5 = (3*Mx*kr*lxp + 3*My*kr*lyp1 + 3*My*kr*lyp2 + 3*Mz*lxp*lyp1 + R*kr*lxp*lyp1 + R*kr*lxp*lyp2)/(6*kr*lxp*(lyp1 + lyp2));
    T6 = R/6;
 
    
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

