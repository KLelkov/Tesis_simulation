close all
figure;
plot(Time, odo_w(:,1)+0.2, 'b', 'LineWidth', 1.5);
hold on
plot(Time, w(:, 1), 'k', 'LineWidth', 1.0);
grid on
xlabel '�����, �'
ylabel '������� ��������, ���/�'

figure;
plot(Time, odo_gamma, 'b', 'LineWidth', 1.5);
hold on
plot(Time, (gamma(:, 1) + gamma(:,2)) / 2, 'k', 'LineWidth', 1.0);
grid on
xlabel '�����, �'
ylabel '���� ��������, ���'

figure;
plot(Time, gyro_anr + 0.06, 'b', 'LineWidth', 1.5);
hold on
plot(Time, Anr, 'k', 'LineWidth', 1.0);
grid on
xlabel '�����, �'
ylabel '������� ��������, ���/c'

figure;
plot(Time, gps_pos(:,1), 'b', 'LineWidth', 1.5);
hold on
plot(Time, X, 'k', 'LineWidth', 1.0);
grid on
xlabel '�����, �'
ylabel '����������, �'