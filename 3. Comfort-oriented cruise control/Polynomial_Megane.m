%% Generation of comfort-oriented refrence speed

%% Speed for each road profile from 30 to 100
rmsclass_A = [0.05 0.08  0.11  0.13 0.16 0.22 0.28 0.34 0.40 0.46 0.52]
rmsclass_B = [0.11 0.17  0.22  0.26 0.33 0.43 0.56 0.68 0.80 0.92 1.04]
rmsclass_C = [0.21 0.33  0.43  0.52 0.65 0.86 1.11 1.35 1.59 1.83 2.06]
rmsclass_D = [0.42 0.67  0.86  1.04 1.31 1.73 2.22 2.71 3.19 3.67 4.13]
% Vitesse en m/s pour 30 to 100
speed = [30/3.6 40/3.6 50/3.6 60/3.6 70/3.6 80/3.6 90/3.6 100/3.6 110/3.6 120/3.6 130/3.6]
%Plot vitesse
plot(speed, rmsclass_A, 'r', 'LineWidth', 2.0); hold on;     
plot(speed, rmsclass_B, 'b', 'LineWidth', 2.0); hold on;  
plot(speed, rmsclass_C, 'y', 'LineWidth', 2.0); hold on;     
plot(speed, rmsclass_D, 'm', 'LineWidth', 2.0); 
ylabel('RMS ','FontSize', 12);
xlabel('Velocities ','FontSize', 12);
legend('Class A', 'Class B', 'Class C', 'Class D', 'Location', 'northwest');
legend('boxoff');

%% Polynomial
% Class A
x = rmsclass_A;
y = speed;
poli_road_A = polyfit(x, y, 7);
x1 = rmsclass_A;
y1 = polyval(poli_road_A, x1);
figure;
plot(x, y, 'o');
hold on;
plot(x1, y1);
hold off;

% Class B
x2 = rmsclass_B;
y2 = speed;
poli_road_B = polyfit(x2, y2, 7);
x3 = rmsclass_B;
y3 = polyval(poli_road_B, x2);
figure;
plot(x2, y2, 'o');
hold on;
plot(x3, y3);
hold off;

% Class C
x4 = rmsclass_C;
y4 = speed;
poli_road_C = polyfit(x4, y4, 7);
x5 = rmsclass_C;
y5 = polyval(poli_road_C, x4);
figure;
plot(x4, y4, 'o');
hold on;
plot(x5, y5);
hold off;

% Class D
x6 = rmsclass_D;
y6 = speed;
poli_road_D = polyfit(x6, y6, 7);
x7 = rmsclass_D;
y7 = polyval(poli_road_D, x6);
figure;
plot(x6, y6, 'o');
hold on;
plot(x7, y7);
hold off;

%% Plot all
figure;
% plot(x, y, 'o'); hold on;
plot(x1, y1, 'r', 'LineWidth', 2.0); hold on; 
% plot(x2, y2, 'o'); hold on;
plot(x3, y3, 'b', 'LineWidth', 2.0); hold on;
% plot(x4, y4, 'o'); hold on;
plot(x5, y5, 'c', 'LineWidth', 2.0); hold on;
% plot(x6, y6, 'o'); hold on;
plot(x7, y7, 'y', 'LineWidth', 2.0) ;
ylabel('Vehicle speed (m/s)', 'FontSize', 12);
xlabel('Frequency weighted RMS acceleration (m/s^2)', 'FontSize', 12);
% legend('PolA', 'Class A', 'PolB', 'Class B', 'PolC', 'Class C', 'PolD', 'Class D', 'Location', 'northwest');
legend('Class A', 'Class B', 'Class C', 'Class D', 'Location', 'southeast');
legend('boxoff');
grid on;
title('Vehicle speed as a function of frequency weighted RMS acceleration');
set(gca, 'FontSize', 20);