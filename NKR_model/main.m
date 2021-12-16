[fX, fY, fHeading, fVelocity, fRate, fBetta, state] = test_navigation(Time, odo_gamma, gyro_anr, gps_pos, gps_vel, odo_w);

limit = 5000;
close all
figure
plot(fY(1:limit),fX(1:limit), 'b');
axis equal
grid on
hold on
plot(Y(1:limit),X(1:limit), 'k');
title('Trajectory');

figure;
plot(Time(1:limit), fHeading(1:limit), 'b')
title('Heading')
grid on
hold on
plot(Time(1:limit), Heading(1:limit), 'k')
plot(Time(1:limit), fHeading(1:limit) - state(1:limit, 5), 'r')

legend raw real corrected

for i = 1:limit
    vxr(i) = V(i) *cos(Heading(i));
end

figure;
plot(Time(1:limit), fVelocity(1:limit,1), 'b')
title('Velocity')
grid on
hold on
plot(Time(1:limit), vxr, 'k')
plot(Time(1:limit), fVelocity(1:limit,1) - state(1:limit, 3), 'r')
plot(Time(1:limit), state(1:limit, 3), 'g')
legend raw real corrected

figure;
plot(Time(1:limit), fRate(1:limit), 'b')
title('Angular Rate')
grid on
hold on
plot(Time(1:limit), Anr(1:limit), 'k')
plot(Time(1:limit), fRate(1:limit) - state(1:limit, 6), 'r')
legend raw real corrected
plot(Time(1:limit), gyro_anr(1:limit), 'c')

figure;
plot(Time(1:limit), -state(1:limit, 13), 'b')
title('Betta')
grid on
hold on
plot(Time(1:limit), -state(1:limit, 14), 'g')
plot(Time(1:limit), -state(1:limit, 15), 'r')
plot(Time(1:limit), -state(1:limit, 16), 'm')
plot(Time(1:limit), Betta(1:limit, 1), 'k')