clc
state.xg = 0;
state.yg = 0;
state.zg = -10;
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
state.pitch = 0.2;
state.roll = 0;
state.dyaw = 0;
state.dpitch = 0.6;
state.droll = 0;
state.dwxb = 0;
state.dwyb = 0;
state.dwzb = 0;
state.dxg = 0;
state.dyg = 0;
state.dzg = 0;

simTime = 16;
dt = 0.01;
nSim = simTime / dt;
Position = zeros(nSim, 3);
Orientation = zeros(nSim, 3);
Velocity = zeros(nSim, 3);
Rate = zeros(nSim, 3);
dRate = zeros(nSim, 3);
Controls = zeros(nSim, 6);
Time = zeros(nSim, 1);

w = 1000 - 100;
controls = [w w w-3 w-3 w w];
target.yaw = 0;
target.dvx = 0;
target.dvy = 0;
target.dvz = 0;
target.dpitch = 0;
target.dyaw = 0.0;
target.pitch = 0.0;
control_params.last_dp = 0;
for i = 1:nSim
    [controls, control_params] = uav_control(state, controls, target, control_params);
    state = uav_sim_step(state, controls);
    
    Time(i) = i * dt;
    Position(i,:) = [state.xg, state.yg, state.zg];
    Orientation(i,:) = [state.yaw, state.pitch, state.roll];
    Velocity(i,:) = [state.dxg, state.dyg, state.dzg];
    Rate(i,:) = [state.dyaw, state.dpitch, state.droll];
    Controls(i,:) = controls;
    dRate(i,:) = [state.dwxb, state.dwyb, state.dwzb];
end


close all
figure
plot(Time, Position);
grid on
title 'Position'

figure
plot(Time, Velocity);
grid on
title 'Velocity'

figure
plot(Time, Orientation);
grid on
title 'Orientation'

figure
plot(Time, Rate);
grid on
title 'Rate'

figure
plot(Time, dRate);
grid on
title 'dRate'

figure
plot(Time, Controls);
grid on
title 'Controls'