clc
%% Trajectory
% X Y Z Xpoi Ypoi
target_points = [0, 0.0, 3, 0, 3; % 1
    0, 0.0, 5, 0, 3; % 1
    -4.5, 2, 5, 0, 3; % 1
    -4.5, 7, 5, 0, 7; % 2
    -3.5, 10, 5, 0, 12; % 3
    -3.5, 10, 2, -5, 12; % 4
%     -5.5, 9, 2, -5, 12; % 4
    -8, 11, 2, -5, 12; % 4
    -8, 12.5, 5, -7, 16; % 5
    -10, 14, 5, -7, 16; % 5
    -15, 16, 5, -14, 20; % 6
    -19.5, 20, 5, -14, 20; % left edge
    -19.5, 24, 5, -14, 20; % 6
    -16.5, 24, 5, -14, 20; % 6
    -13.5, 23, 5, -7, 16; % 5
    -10.5, 22.5, 5, -7, 16; % 5
    -7, 21.5, 5, -7, 16; % 5
    -4, 20, 5, 0, 20; % 7
    -4.5, 25, 5, 0, 27; % 8
    -4.5, 30, 5, -2, 34; % 9
    -4, 31, 9, -2, 34; % 9
    -6, 31, 7, -2, 34; % 9
    -8, 33, 7, -2, 34; % 9 
    -8, 38, 7, -2, 34; % 9
    -4, 38, 7, 0, 27; % 8
    -0, 38, 9, 0, 27; % tail upper ---------
    -0, 38, 5, 0, 27;% tail lower  -----
    4, 38, 7, 0, 27; % 8            
    8, 38, 7, 2, 34; % 10
    8, 33, 7, 2, 34; % 10
    6, 31, 7, 2, 34; % 10
    4, 31, 9, 2, 34; % 10
    4.0, 31, 5, 2, 34; % 10
    4.5, 25, 5, 0, 27; % 8
    4, 20, 5, 0, 20; % 7
    7, 21.5, 5, 7, 16; % 11
    10.5, 22.5, 5, 7, 16; % 11
    13.5, 23, 5, 7, 16; % 11
    16.5, 24, 5, 14, 20; % 12
    19.5, 24, 5, 14, 20; % 12
    19.5, 20, 5, 14, 20; % right edge
    15, 16, 5, 14, 20; % 12
    10, 14, 5, 7, 16; % 11
    8, 12.5, 5, 7, 16; % 11
    8, 12.5, 2, 5, 12; % 13
    5.5, 9, 2, 5, 12; % 13
    3.5, 10, 5, 0, 12; % 3
    4.5, 7, 5, 0, 7; % 2
    4.5, 2, 5, 0, 3; % 
    4.5, 2, 3, 0, 3; % 1
    0, 0, 3, 0, 3; % 1
    0, 0, 0, 0, 3; % 1
    ];

% adjust frame for lazy people!
temp1 = target_points(:,1);
temp2 = target_points(:,2);
temp3 = target_points(:,4);
temp4 = target_points(:,5);
target_points(:,1) = temp2;
target_points(:,2) = temp1;
target_points(:,4) = temp4;
target_points(:,5) = temp3;

%% Sim setup
state.xg = 0;
state.yg = 0;
state.zg = 0; % remember that it points down!
state.dxb = 0;
state.dyb = 0;
state.dzb = 0;
state.ddxb = 0;
state.ddyb = 0;
state.ddzb = 0;
state.wxb = 0;
state.wyb = 0;
state.wzb = 0;
state.yaw = 0;
state.pitch = 0;
state.roll = 0;
state.dyaw = 0;
state.dpitch = 0.0;
state.droll = 0;
state.dwxb = 0;
state.dwyb = 0;
state.dwzb = 0;
state.dxg = 0;
state.dyg = 0;
state.dzg = 0;
state.ddxg = 0;
state.ddyg = 0;
state.ddzg = 0;

simTime = 500;
dt = 0.01;
nSim = simTime / dt;
Position = zeros(nSim, 3);
Orientation = zeros(nSim, 3);
Velocity = zeros(nSim, 3);
Acceleration = zeros(nSim, 3);
Rate = zeros(nSim, 3);
dRate = zeros(nSim, 3);
Controls = zeros(nSim, 6);
Controls_tr = zeros(nSim, 4);
Time = zeros(nSim, 1);

w = 1000 - 100;
controls = [w w w w w w];
target.yaw = 0.0;
target.forward_vel = 0.0;
target.lateral_vel = 0.0;
target.dvy = 0.0;
target.dvz = 0;
target.dpitch = 0.0;
target.dyaw = 0.0;
target.pitch = 0.0;


% target.pitch = 0.0;
control_params.h_int = 0;
control_params.z_int = 0;
control_params.dist_hor = 0;
control_params.last_cte = 0;

%% Main cycle
mission = [0, 0, 0];

wp_count = 1;
control_params.last_x = 0;
control_params.last_y = 0;
control_params.last_z = 0;
wp.x = target_points(wp_count, 1);
wp.y = target_points(wp_count, 2);
wp.z = -target_points(wp_count, 3);
mission(wp_count+1,:) = [wp.x, wp.y, wp.z];
for i = 1:nSim
    [traj_ctrl, control_params, reached] = uav_trajectory_control(state, wp, control_params);
    if (reached)
        disp('reached!')
        control_params.last_x = wp.x;
        control_params.last_y = wp.y;
        control_params.last_z = wp.z;
        wp_count = wp_count + 1;
        if (wp_count > length(target_points))
            break;
        end
        wp.x = target_points(wp_count, 1);
        wp.y = target_points(wp_count, 2);
        wp.z = -target_points(wp_count, 3);
        mission(wp_count+1,:) = [wp.x, wp.y, wp.z];
        [traj_ctrl, control_params, reached] = uav_trajectory_control(state, wp, control_params);
%         break;
    end
    target.dyaw = traj_ctrl.dheading;
    target.dvz = traj_ctrl.height;
    target.forward_vel = traj_ctrl.forward;
    target.lateral_vel = traj_ctrl.lateral;
    [controls, control_params] = uav_locomotion_control(state, controls, target, control_params);
    state = uav_sim_step(state, controls);
    
    Time(i) = i * dt;
    Position(i,:) = [state.xg, state.yg, state.zg];
    Orientation(i,:) = [state.yaw, state.pitch, state.roll];
    Velocity(i,:) = [state.dxg, state.dyg, state.dzg];
    Rate(i,:) = [state.dyaw, state.dpitch, state.droll];
    Controls(i,:) = controls;
    Controls_tr(i,:) = [target.dyaw, target.forward_vel, target.lateral_vel, target.dvz];
    dRate(i,:) = [state.dwxb, state.dwyb, state.dwzb];
    Acceleration(i,:) = [state.ddxg, state.ddyg, state.ddzg];
end

%% Graphs
close all
figure('Name', 'Motion parameters', 'Position', [100 150 1750 650])
subplot(3,3,[1 4 7])
plot3(Position(:,2), Position(:,1), -Position(:,3), 'Linewidth', 1)
grid on
hold on
plot3(Position(end,2), Position(end,1), -Position(end,3), '.r', 'MarkerSize', 15)
plot3(mission(:,2), mission(:,1), -mission(:,3), 'k', 'Linewidth', 1)
% Get rid of infinite small axes, by changing the limits
xl = get(gca, 'XLim');
yl = get(gca, 'YLim');
zl = get(gca, 'ZLim');
if abs(xl(2) - xl(1)) < 0.01
    xlim([xl(1)-1, xl(1)+1]);
end
if abs(yl(2) - yl(1)) < 0.01
    ylim([yl(1)-1, yl(1)+1]);
end
if abs(zl(2) - zl(1)) < 0.01
    zlim([zl(1)-1, zl(1)+1]);
end
axis equal
xlabel 'Yg (east)'
ylabel 'Xg (north)'
zlabel 'Zg (down), reversed'
subplot(3,3,2)
plot(Time, Position(:,1:2), 'LineWidth', 2);
grid on
hold on
plot(Time, -Position(:,3), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
title 'Position'
xlabel 'Time, s'
ylabel 'Coords, m'
legend xg yg zg(rev)
subplot(3,3,5)
plot(Time, Velocity(:,1:2), 'LineWidth', 2);
grid on
hold on
plot(Time, -Velocity(:,3), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
title 'Velocity'
xlabel 'Time, s'
ylabel 'Velocity, m/s'
legend Vxg Vyg Vzg(rev)
subplot(3,3,8)
plot(Time, Acceleration(:,1:2), 'LineWidth', 2);
grid on
hold on
plot(Time, -Acceleration(:,3), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
title 'Acceleration'
xlabel 'Time, s'
ylabel 'Acceleration, m/s2'
legend dVxg dVyg dVzg(rev)


subplot(3,3,3)
plot(Time, Orientation .* 180.0 / pi, 'LineWidth', 2);
grid on
title 'Orientation'
xlabel 'Time, s'
ylabel 'Angle, deg'
legend Yaw Pitch Roll

subplot(3,3,6)
plot(Time, Rate .* 180.0 / pi, 'LineWidth', 2);
grid on
title 'Rate'
xlabel 'Time, s'
ylabel 'Rate, deg/s'
legend dYaw dPitch dRoll

subplot(3,3,9)
plot(Time, dRate .* 180.0 / pi, 'LineWidth', 2);
grid on
title 'Angular acceleration'
xlabel 'Time, s'
ylabel 'dRate, deg/s2'
legend dwxb dwyb dwzb

figure('Name', 'Control parameters', 'Position', [100 150 1050 650])
subplot(3,2,1)
plot(Time, Controls, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
grid on
hold on
plot(Time, Controls(:,6), 'r', 'LineWidth', 2);
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'W6'
subplot(3,2,2)
plot(Time, Controls, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
grid on
hold on
plot(Time, Controls(:,1), 'b', 'LineWidth', 2);
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'W1'

subplot(3,2,3)
plot(Time, Controls, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
grid on
hold on
plot(Time, Controls(:,5), 'b', 'LineWidth', 2);
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'W5'

subplot(3,2,4)
plot(Time, Controls, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
grid on
hold on
plot(Time, Controls(:,2), 'r', 'LineWidth', 2);
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'W2'

subplot(3,2,5)
plot(Time, Controls, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
grid on
hold on
plot(Time, Controls(:,4), 'b', 'LineWidth', 2);
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'W4'

subplot(3,2,6)
plot(Time, Controls, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
grid on
hold on
plot(Time, Controls(:,3), 'r', 'LineWidth', 2);
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'W3'