[fX, fY, fHeading, fVelocity, fRate, fBetta] = test_navigation(Time, odo_gamma, gyro_anr, gps_pos, gps_vel, odo_w);

limit = 1000;
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

figure;
plot(Time(1:limit), fVelocity(1:limit), 'b')
title('Velocity')
grid on
hold on
plot(Time(1:limit), V(1:limit), 'k')

figure;
plot(Time(1:limit), fRate(1:limit), 'b')
title('Angular Rate')
grid on
hold on
plot(Time(1:limit), Anr(1:limit), 'k')

figure;
plot(Time(1:limit), fBetta(1:limit, 1), 'b')
title('Betta')
grid on
hold on
plot(Time(1:limit), Betta(1:limit, 1), 'k')