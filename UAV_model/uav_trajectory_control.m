function [controls, control_params, reached] = uav_trajectory_control(state, target, control_params)
    reached = false;
    dt = 0.01;
    
    xg = state.xg;
    yg = state.yg;
    zg = state.zg;
    dxg = state.dxg;
    dyg = state.dyg;
    dzg = state.dzg;
    
    yaw = state.yaw;
    dyaw = state.dyaw;
    pitch = state.pitch;
    roll = state.roll;
    
    %% Heading error
    dx = target.x - xg;
    dy = target.y - yg;
    
    target_h = atan2(dy, dx);
    he = pi2pi(target_h - yaw);
    % Yaw difference -> dYaw control
    controls.dheading = 2.78 * he - 0.21 * dyaw;
%     controls.dheading = target_h;
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
    controls.height = 0.26 * ze  + 0.12 * dzg;
    
end

