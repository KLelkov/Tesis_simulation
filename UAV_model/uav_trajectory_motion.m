clear
close all
% X Y Z Xpoi Ypoi
target_points = [0, 0, 3, 0, 3; % 1
    0, 0, 5, 0, 3; % 1
    -4.5, 2, 5, 0, 3; % 1
    -4.5, 7, 5, 0, 7; % 2
    -3.5, 10, 5, 0, 12; % 3
    -5.5, 9, 2, -5, 12; % 4
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
    4.5, 30, 5, 2, 34; % 10
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
    8, 11, 2, 5, 12; % 13
    5.5, 9, 2, 5, 12; % 13
    3.5, 10, 5, 0, 12; % 3
    4.5, 7, 5, 0, 7; % 2
    4.5, 2, 5, 0, 3; % 1
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

uav_vel = 1.0 / sqrt(3); % m/s
uav_rate = 5.0; % rad/s
uav_acc_lim = 1.4; % m/s
uav_rcc_lim = 4; % rad/s
dt = 0.1; % sec
X = 0;
Y = 0;
Z = 0;
Velocity(1,:) = [0, 0, 0];
Acceleration(1,:) = [0, 0, 0];
Acceleration_b(1,:) = [0, 0, 0];
Heading = 0;
dHeading = 0;
ddHeading = 0;
Time = 0;
i = 1;
pnt = 1;
fin = false;
while ~fin
% for iw = 1:400
    i = i + 1;
    dx = target_points(pnt,1) - X(end);
    dy = target_points(pnt,2) - Y(end);
    dz = target_points(pnt,3) - Z(end);
    dx1 = target_points(pnt,4) - X(end);
    dy1 = target_points(pnt,5) - Y(end);
%     target_heading = atan2(dy1, dx1);
%     dh = target_heading - Heading(end);
    dh =  atan2(dy, dx) - Heading(end) + pi/2;
    if dx == 0 && dy == 0
        dh =  0 - Heading(end);
    end
    
    
    
    dx = bound(2*dx, -uav_vel, uav_vel);
    dy = bound(2*dy, -uav_vel, uav_vel);
    dz = bound(2*dz, -uav_vel, uav_vel);
    dh = bound(2*dh, -uav_rate, uav_rate) * sign(dx);
%     dh = 1;
    if sqrt(dx^2 + dy^2 + dz^2) < 0.2
        pnt = pnt + 1;
        if pnt > 50
            fin = true;
        end
    end
    
    if abs(dh) > pi
        dh = dh - 2*pi*sign(dh);
    end
    
    
    Acceleration(i,1) = (dx - Velocity(end,1)) * 1.2 + Acceleration(i-1,1)*0.2;
    Acceleration(i,2) = (dy - Velocity(end,2)) * 1.2 + Acceleration(i-1,2)*0.2;
    Acceleration(i,3) = (dz - Velocity(end,3)) * 1.2 + Acceleration(i-1,3)*0.2;
    Acceleration(i,1) = bound(Acceleration(i,1), -uav_acc_lim, uav_acc_lim);
    Acceleration(i,2) = bound(Acceleration(i,2), -uav_acc_lim, uav_acc_lim);
    Acceleration(i,3) = bound(Acceleration(i,3), -uav_acc_lim, uav_acc_lim);
 
    ddHeading(i) = (dh - dHeading(end)) * 0.9 + ddHeading(i-1) * 0.2;
    ddHeading(i) = bound(ddHeading(i), -uav_rcc_lim, uav_rcc_lim);
     
    Time(i) = Time(i-1) + dt;
    X(i) = X(i-1) + Velocity(i-1,1)*dt;
    Y(i) = Y(i-1) + Velocity(i-1,2)*dt;
    Z(i) = Z(i-1) + Velocity(i-1,3)*dt;
    Velocity(i,1) = Velocity(i-1,1) + Acceleration(i-1,1) * dt;
    Velocity(i,2) = Velocity(i-1,2) + Acceleration(i-1,2) * dt;
    Velocity(i,3) = Velocity(i-1,3) + Acceleration(i-1,3) * dt;
    Heading(i) = Heading(i-1) + dHeading(i-1) * dt;
    dHeading(i) = dHeading(i-1) + ddHeading(i-1) * dt;
    
    
%     Time(i) = Time(i-1) + dt;
%     X(i) = X(i-1) + dx*dt;
%     Y(i) = Y(i-1) + dy*dt;
%     Z(i) = Z(i-1) + dz*dt;
% %     Heading(i) = atan2(dy, dx) + pi/2;
%     Heading(i) = Heading(i-1) + dh * dt;
%     
% %     dHeading(i) = (Heading(i) - Heading(i-1)) / dt;
%     dHeading(i) = dh;
%     Velocity(i,:) = [dx, dy, dz];
%     Acceleration(i,1) = (Velocity(i,1) - Velocity(i-1,1)) / dt;
%     Acceleration(i,2) = (Velocity(i,2) - Velocity(i-1,2)) / dt;
%     Acceleration(i,3) = (Velocity(i,3) - Velocity(i-1,3)) / dt;
%     
    Acceleration_b(i,1) = Acceleration(i,1) * cos(Heading(i)) + Acceleration(i,2) * sin(Heading(i));
    Acceleration_b(i,2) = -Acceleration(i,1) * sin(Heading(i)) + Acceleration(i,2) * cos(Heading(i));
    Acceleration_b(i,3) = (Velocity(i,3) - Velocity(i-1,3)) / dt;
    
    if abs(Heading(i)) > pi
        Heading(i) = Heading(i) - 2*pi*sign(Heading(i));
    end
    
    Heading(i) = atan2(dy, dx) + pi/2;
    if dx == 0 && dy == 0
        Heading(i) = 0;
    end
end

figure;
plot3(Y, X, Z, 'k');
axis equal
grid on

% figure;
% plot(Time, Heading, 'b', 'LineWidth', 2);
% grid on;
% title 'Heading'

%% Vector animation
len = length(X);
t = len / 10 / 60
vect = 3;
l = line([0 0], [0 0], [0 0]);
for i = 1:len
    %delete(l)
    dx = X(i) + vect * cos(Heading(i));
    dy = Y(i) + vect * sin(Heading(i));
    l = line([Y(i) dy], [X(i) dx], [Z(i) Z(i)], 'Color', 'r', 'LineWidth', 2);
%     drawnow;
%     pause(0.1);
end
hold on
plot3(Y, X, Z, 'k', 'LineWidth', 2);

figure;plot(Heading)
figure;plot(dHeading)
% figure;plot(Heading)

