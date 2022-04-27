function [controls, control_params, reached] = uav_trajectory_control(state, target, control_params)
    reached = false;
    reached_hor = false;
    reached_ver = false;
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
%     if abs(dx) < 0.01
%         dx = 0;
%     end
%     if abs(dy) < 0.01
%         dy = 0;
%     end
    target_h = atan2(dy, dx);
    he = pi2pi(target_h - yaw);
    % Yaw difference -> dYaw control
    controls.dheading = 2.78 * he - 0.21 * dyaw;
%     controls.dheading = target_h;
    %% Crosstrack error
    cdx = target.x - control_params.last_x ;
    cdy = target.y - control_params.last_y;
    
    crelx = xg - control_params.last_x;
    crely = yg - control_params.last_y;
    % End of line criteria
    u = (crelx * cdx + crely * cdy) / (cdx * cdx + cdy * cdy);
    if u > 1
        % Switch
        reached_hor = true;
    end
    CTE = - (crely * cdx - crelx * cdy) / (cdx * cdx + cdy * cdy);
    if ((cdx == 0.0) && (cdy == 0)) 
        controls.dheading = 0;
        CTE = 0;
        reached_hor = true;
    end
    if CTE < 0.01
        CTE = 0;
    end
    %% Forward velocity
    cdx_n1 = -(target.x - control_params.last_x) / (cdx);
    cdy_n1 = -(target.y - control_params.last_y) / abs(cdy);
    dist_hor = sqrt(dx*dx + dy*dy);
    if abs(he) > pi/3
        dist_hor = - 1.0 * dist_hor;
    end
%     overshoot = false;
%     if (cdx > 0)
%     else
%         crelx >= cdx
%     end
%     if (crelx >= cdx && crely >= cdy) %|| (crelx <= cdx_n1 && crely <= cdy_n1)
%         dist_hor = -dist_hor;
%     end
    fve = dist_hor * 2.5 + 0.7 * (dist_hor - control_params.dist_hor) / dt;
    fve = bound(fve, -0.8, 0.8);
    control_params.dist_hor = dist_hor;
    controls.forward = fve;
    %% Lateral velocity
    controls.lateral = 6.8 * CTE - 00.0 * (CTE - control_params.last_cte) / dt;
    controls.lateral = bound(controls.lateral, -0.8, 0.8);
    control_params.last_cte = CTE;
    %% Vertical velocity
    ze = target.z - zg;
    controls.height = 0.26 * ze  + 0.12 * dzg;
    controls.height = bound(controls.height, -2.8, 2.8);
    
    %% Alternative reached condition
    if abs(ze) < 0.1
        reached_ver = true;
    end
    if reached_hor && reached_ver
        reached = true;
    end
end

