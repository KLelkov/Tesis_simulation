function [controls, control_params] = uav_control(state, controls, target, control_params)
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
    vz_control = 13.3 * vze - 4.25 * dVg(3);
    
    % Forward velocity
    vxe = target.dvx - dxg;
    vx_control = -0.76 * vxe + 0.27 * dVg(1);
    
    % Lateral velocity
    vye = target.dvy - dyg;
    vy_control = 0.6 * vye - 0.2 * dVg(2);
    
    % Pitch
    pe1 = vx_control - pitch;
%     pe1 = target.pitch - pitch;
    dp_control = 1.7 * pe1 - 0.14 * dpitch;
%     pe = target.dpitch - dpitch;
    pe = dp_control - dpitch;
    ddp = ((1.3 * (target.pitch - pitch) - 1.1 * dpitch) - control_params.last_dp) * 0.01;
    last_dp = (1.3 * (target.pitch - pitch) - 1.1 * dpitch);
%     pitch_control = 0.0003 * (1.3 * (target.pitch - pitch) - 1.1 * dpitch) - 0.0001 * dwyb;
    ddp = ((0.3 - dpitch) - control_params.last_dp)/0.01;
%     ddp = bound(ddp, -20, 20);
%     control_params.last_dp = (0.0 - dpitch);
%     err = (0.3 - dpitch) - 0.5*dwyb;
    pitch_control = 0.42 * pe - 0.02*dwyb;
%     if abs(err) > 0.02
%         pitch_control = 0.0006 * (0.3 - dpitch) + 0.0002 * ddp;
%     else
%         pitch_control = 0.0006 * (0.3 - dpitch);
%     end
    
%     if abs(pitch_control) < 0.00001
%         pitch_control = 0.0;
%     end
%     pitch_control = bound(pitch_control, -5e-5, 5e-5);
%     dpitch
%     dwyb
%     ddp
   
    % Roll
    re = vy_control - roll;
    roll_control = 1.3 * re + 0.1 * wxb;
    
    % Yaw
    ye = target.dyaw - dyaw;
    yaw_control = 0.03 * ye + 0.002 * dwzb;
    yaw_control  = 0.01;
    
    % Convert control moments into separate motors lift force change
    Mx = 0;%roll_control;
    My = pitch_control;
    Mz = 0;
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

