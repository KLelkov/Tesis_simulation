% sys(1) = 4e-5; % state.pos
% sys(2) = 1e-5; % state.h
% sys(3) = 1e-4; % state.v
% sys(4) = 2e-5; % state.dh
% sys(5) = 1e-4; % state.odo
% 
% meas(1) = 4e0;  % gps_pos
% meas(2) = 8e-1; % pgs_ve;
% meas(3) = 4e-1; % gyro
% meas(4) = 3e-1; % odo

sys(1) = 2e-5; % state.pos
sys(2) = 1e-4; % state.h
sys(3) = 3e-3; % state.v
sys(4) = 8e-4; % state.dh
sys(5) = 1e-2;%8e-3; % state.odo

meas(1) = 3e0;  % gps_pos
meas(2) = 6e-1; % pgs_ve;
meas(3) = 4e-1; % gyro
meas(4) = 1e0;%8e-1; % odo


emean = zeros(75, 1);
ediv = zeros(75,1);
for i=1:75
    [emean(i), ediv(i)] = simple_model_fun(sys, meas);
end
% disp(sys);
% disp(meas);
fprintf(['sys:' repmat(' %.5f ',1,numel(sys)) '\n'],sys)
fprintf(['meas:' repmat(' %.2f ',1,numel(meas)) '\n'],meas)
fprintf('error mean: %.2f std: %.2f\n', mean(emean), mean(ediv))