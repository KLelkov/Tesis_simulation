clc
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

simTime = 10;
dt = 0.01;
nSim = simTime / dt;
Position = zeros(nSim, 3);
Orientation = zeros(nSim, 3);
Velocity = zeros(nSim, 3);
Acceleration = zeros(nSim, 3);
Rate = zeros(nSim, 3);
dRate = zeros(nSim, 3);
Controls = zeros(nSim, 6);
Time = zeros(nSim, 1);

w = 1000 - 100;
controls = [w w w w w w];
target.yaw = 0.0;
target.dvx = 0.0;
target.dvy = 1.5;
target.dvz = 0;
target.dpitch = 0.0;
target.dyaw = 0.0;
target.pitch = 0.0;
wp.x = 20;
wp.y = 0;
wp.z = -10;
% target.pitch = 0.0;
control_params.last_dp = 0;
for i = 1:nSim
    [controls, control_params] = uav_locomotion_control(state, controls, target, control_params);
    state = uav_sim_step(state, controls);
    
    Time(i) = i * dt;
    Position(i,:) = [state.xg, state.yg, state.zg];
    Orientation(i,:) = [state.yaw, state.pitch, state.roll];
    Velocity(i,:) = [state.dxg, state.dyg, state.dzg];
    Rate(i,:) = [state.dyaw, state.dpitch, state.droll];
    Controls(i,:) = controls;
    dRate(i,:) = [state.dwxb, state.dwyb, state.dwzb];
    Acceleration(i,:) = [state.ddxg, state.ddyg, state.ddzg];
end


close all
figure('Name', 'Motion parameters', 'Position', [100 150 1750 650])
subplot(3,3,[1 4 7])
plot3(Position(:,2), Position(:,1), -Position(:,3), 'Linewidth', 1)
grid on
hold on
plot3(Position(end,2), Position(end,1), -Position(end,3), '.r', 'MarkerSize', 15)
% axis equal
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
legend xg yg zg
subplot(3,3,5)
plot(Time, Velocity(:,1:2), 'LineWidth', 2);
grid on
hold on
plot(Time, -Velocity(:,3), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
title 'Velocity'
xlabel 'Time, s'
ylabel 'Velocity, m/s'
legend Vxg Vyg Vzg
subplot(3,3,8)
plot(Time, Acceleration(:,1:2), 'LineWidth', 2);
grid on
hold on
plot(Time, -Acceleration(:,3), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
title 'Acceleration'
xlabel 'Time, s'
ylabel 'Acceleration, m/s2'
legend dVxg dVyg dVzg


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