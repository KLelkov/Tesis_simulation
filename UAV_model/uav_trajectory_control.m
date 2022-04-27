function [controls, control_params, reached] = uav_trajectory_control(state, target, control_params)
    reached = false;

    %W = controls;
    
    
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
    dt = 0.01;
    
    
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

    
    % Propulsion moments on motors
%     T = zeros(1,6);
%     for j = 1:6
%         vzi = k * W(j)^2;
%         T(j) = 2 * ro * pi * r^2 * vzi * (vzi - dzb);
%     end
    %% Heading error
    dx = target.x - xg;
    dy = target.y - yg;
    
    target_h = atan2(dy, dx);
    he = pi2pi(target_h - yaw);
%     controls.heading = 1.2 * he + 0.04 * control_params.h_int;
    controls.heading = target_h;
    control_params.h_int = control_params.h_int + he * dt;
    %% Crosstrack error
    cdx = target.x - control_params.last_x;
    cdy = target.y - control_params.last_y;
    crelx = xg - control_params.last_x;
    crely = yg - control_params.last_y;
    % End of line criteria
    u = (crelx * cdx + crely * cdy) / (cdx * cdx + cdy * cdy);
    if u > 1
        % Switch
        reached = true;
    end
    CTE = - (crely * cdx - crelx * cdy) / (cdx * cdx + cdy * cdy);
    %% Forward velocity
    dist_hor = sqrt(dx*dx + dy*dy);
    fve = dist_hor * 1.5;
    fve = bound(fve, -2, 2);
    controls.forward = fve;
    %% Lateral velocity
    controls.lateral = 6.8 * CTE - 30.0 * (CTE - control_params.last_cte) / dt;
    controls.lateral = bound(controls.lateral, -2, 2);
    control_params.last_cte = CTE;
    %% Vertical velocity
    ze = target.z - zg;
    controls.height = 0.26 * ze + 0.0 * control_params.z_int + 0.12 * dzg;
    control_params.z_int = control_params.z_int + ze * dt;
    

    
end

