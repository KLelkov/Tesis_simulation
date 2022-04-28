gyro = Rate(1:limit,:);
accl = Acceleration_body(1:limit,:);
gnss = Position(1:limit,:);
baro = Position(1:limit,3);
truePos = Position(1:limit,:);
trueAng = Orientation(1:limit,:);

d = 0.1;

% Generate GNSS errors
gps_angle = pi/6; % ?
mu = 0.1;
delta = 2.2;
sigma_w = 2.1;
sigma = 1.8;
gps_delta(i-1,:) = [0, 0, 0];
gps_ddelta(i-1,:) = [0, 0, 0];
for i = 2:length(gyro)
    gps_delta(i,1) = gps_delta(i-1,1) + gps_ddelta(i-1,1) * dt;
    gps_delta(i,2) = gps_delta(i-1,2) + gps_ddelta(i-1,2) * dt;
    gps_delta(i,3) = gps_delta(i-1,3) + gps_ddelta(i-1,3) * dt;
    
    gps_ddelta(i,1) = - mu * gps_delta(i,1) + sqrt(2 * (sigma*cos(gps_angle))^2 * mu) * normrnd(0,1);
    gps_ddelta(i,2) = - mu * gps_delta(i,2) + sqrt(2 * (sigma*sin(gps_angle))^2 * mu) * normrnd(0,1);
    gps_ddelta(i,3) = - mu * gps_delta(i,3) + sqrt(2 * (sigma*1.4)^2 * mu) * normrnd(0,1);

    % Scientific GNSS errors
    gnss(i,1) = gnss(i,1) + delta*cos(gps_angle) + normrnd(0, sigma_w) + gps_delta(i,1);
    gnss(i,2) = gnss(i,2) + delta*sin(gps_angle) + normrnd(0, sigma_w) + gps_delta(i,2);
    gnss(i,3) = gnss(i,3) + delta + normrnd(0, sigma_w*1.4) + gps_delta(i,3);
end

% Generate GYRO errors
gyro_noise = 3*pi/180; % rad/s
gyro_bias = [normrnd(0, 0.05), normrnd(0, 0.05), normrnd(0, 0.05)]; % rad/s
for i = 1:length(gyro)
    gyro(i,1) = gyro(i,1) + gyro_bias(1) + normrnd(0, gyro_noise);
    gyro(i,2) = gyro(i,2) + gyro_bias(2) + normrnd(0, gyro_noise);
    gyro(i,3) = gyro(i,3) + gyro_bias(3) + normrnd(0, gyro_noise);
end

% Generate ACCL errors
accl_noise = 0.8; % m/s
accl_bias = [normrnd(0, 0.03), normrnd(0, 0.03), normrnd(0, 0.03)]; % rad/s
for i = 1:length(gyro)
    yaw = trueAng(i,1);
    pitch = trueAng(i,2);
    roll = trueAng(i,3);
    Rgb = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch);
        cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), cos(pitch)*sin(roll);
        cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), cos(pitch)*cos(roll)];
    add = Rgb * [0; 0; 9.8];
    accl(i,1) = accl(i,1) + accl_bias(1) + normrnd(0, accl_noise) + add(1);
    accl(i,2) = accl(i,2) + accl_bias(2) + normrnd(0, accl_noise) + add(2);
    accl(i,3) = accl(i,3) + accl_bias(3) + normrnd(0, accl_noise) + add(3);
end

% Generate BARO errors
baro_noise = 0.8; % m
baro_bias = normrnd(0, 2.5); % rad/s
for i = 1:length(gyro)
    baro(i) = baro(i) + baro_bias + normrnd(0, baro_noise);
end

clearvars -except gyro accl truePos trueAng baro