clear
close all
% X Y Z Xpoi Ypoi
target_points = [0, -2.5, 3, 0, 3; % 1
    0, -2, 5, 0, 3; % 1
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
    0, -2.5, 3, 0, 3; % 1
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

uav_vel = 1.0/ sqrt(3); % m/s
uav_rate = 1.0; % rad/s
dt = 0.1; % sec
X = 0;
Y = 0;
Z = 0;
Heading = 0;
Time = 0;
i = 1;
pnt = 1;
fin = false;
while ~fin
    i = i + 1;
    dx = target_points(pnt,1) - X(end);
    dy = target_points(pnt,2) - Y(end);
    dz = target_points(pnt,3) - Z(end);
    dx1 = target_points(pnt,4) - X(end);
    dy1 = target_points(pnt,5) - Y(end);
    target_heading = atan2(dy1, dx1);
    dh = target_heading - Heading(end);
    
    if abs(dh) > pi
        dh = dh - 2*pi*sign(dh);
    end
    
    dx = bound(3*dx, -uav_vel, uav_vel);
    dy = bound(3*dy, -uav_vel, uav_vel);
    dz = bound(3*dz, -uav_vel, uav_vel);
    dh = bound(2*dh, -uav_rate, uav_rate);
    
    if sqrt(dx^2 + dy^2 + dz^2) < 0.2
        pnt = pnt + 1;
        if pnt > 50
            fin = true;
        end
    end
    
    Time(i) = Time(i-1) + dt;
    X(i) = X(i-1) + dx*dt;
    Y(i) = Y(i-1) + dy*dt;
    Z(i) = Z(i-1) + dz*dt;
    Heading(i) = Heading(i-1) + dh*dt;
    
    if abs(Heading(i)) > pi
        Heading(i) = Heading(i) - 2*pi*sign(Heading(i));
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

