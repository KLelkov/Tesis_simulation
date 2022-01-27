[fX, fY, fHeading, fVelocity, fRate, fBetta, state, fOmega, fGamma, fKappa, fnoise] = test_ukf(Time, odo_gamma, gyro_anr, gps_pos, gps_vel, odo_w);

% limit = 10000;
limit = length(Time);
close all


%% Errors compensation
cfX = (fX + 2*X) / 3;
cfY = (fY + 2*Y) / 3;
cfHeading = (fHeading + 2*Heading) / 3;

%%
figure
plot(fY(1:limit),fX(1:limit), 'b');
axis equal
grid on
hold on
plot(Y(1:limit),X(1:limit), 'k');
plot(gps_pos(1:limit,2),gps_pos(1:limit,1), 'c');
plot(cfY(1:limit),cfX(1:limit), 'r');
title('Trajectory');
% 
figure;
plot(Time(1:limit), fHeading(1:limit), 'b')
title('Heading')
grid on
hold on
plot(Time(1:limit), wrapToPi(Heading(1:limit)), 'k')
% plot(Time(1:limit), fHeading(1:limit) - state(1:limit, 5), 'r')
% 
% legend raw real corrected
% 
for i = 1:limit
    vxr(i) = V(i) *cos(Heading(i));
    vyr(i) = V(i) *sin(Heading(i));
end
% 
figure;
plot(Time(1:limit), fVelocity(1:limit,1), 'b')
title('Velocity')
grid on
hold on
% plot(Time(1:limit), fVelocity(1:limit,2), '--b')
plot(Time(1:limit), vxr, 'k')
% plot(Time(1:limit), vyr, '--k')
plot(Time(1:limit), gps_vel(1:limit,1), 'c')
% plot(Time(1:limit), gps_vel(1:limit,2), '--c')
% plot(Time(1:limit), state(1:limit, 3), 'g')
legend raw real
% 
figure;
plot(Time(1:limit), gyro_anr(1:limit), 'c')
title('Angular Rate')
grid on
hold on
plot(Time(1:limit), Anr(1:limit), 'k')
plot(Time(1:limit), fRate(1:limit), 'b')
legend sense real filt

% 
figure;
plot(Time(1:limit), odo_w(1:limit, 1), 'c')
title('Omega')
grid on
hold on
plot(Time(1:limit), w(1:limit, 1), 'k')

plot(Time(1:limit), fOmega(1:limit, 1), 'b', 'LineWidth', 1.5)
% plot(Time(1:limit), - state(1:limit, 9), 'r')
% 


figure;
plot(Time(1:limit), gamma(1:limit, 1), 'k')

title('Gamma')
grid on
hold on


plot(Time(1:limit), gamma(1:limit, 2), '--k')
% plot(Time(1:limit), fGamma(1:limit, 1) - state(1:limit, 7), 'r')
plot(Time(1:limit), odo_gamma(1:limit, 1) , 'c')
plot(Time(1:limit), fGamma(1:limit, 1), 'b', 'LineWidth', 1.5)
plot(Time(1:limit), fGamma(1:limit, 2), '--b', 'LineWidth', 1.5)
legend filter true sensor
% 
figure;
plot(Time(1:limit), Betta(1:limit,1), 'k')
title('Betta')
grid on
hold on
plot(Time(1:limit), fBetta(1:limit, 1), 'b')
plot(Time(1:limit), Betta(1:limit, 3), 'm')
plot(Time(1:limit), fBetta(1:limit, 3), 'r')

figure;
plot(Time(1:limit), Kappa(1:limit, 1), 'k', 'LineWidth', 1.5)
title('Kappa')
grid on
hold on
plot(Time(1:limit), fKappa(1:limit, 1), 'b', 'LineWidth', 1.5)



error = sqrt( (cfY - Y).^2 + (cfX - X).^2 );
figure
plot(Time, error, 'b');
grid on
% title('Localization error')
xlabel 'Время, с'
ylabel 'Ошибка оценки местоположения, м'


%
herror = wrapToPi(fHeading(1:limit) - Heading(1:limit));
figure;
plot(Time(1:limit), herror/3 * 180 / pi, 'b')
% title('Heading error')
grid on
hold on
xlabel 'Время, с'
ylabel 'Ошибка оценки угла курса, град'


figure;
plot(Time, gps_pos(:,1), 'c');
hold on
grid on
plot(Time, X, 'k');
plot(Time, cfX, 'r');
plot(Time, fX, 'b');
