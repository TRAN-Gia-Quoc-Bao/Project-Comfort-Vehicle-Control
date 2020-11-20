%% Comfort-based car speed control - TRAN Gia Quoc Bao

%% Default commands
close all;
clear all;
clc;

%% Values
% Nominal values
mN = 1410; % car 1410 and driver 70
mi = 1480; % car 1410 and driver 70
v0N = 20;
CrN = 0.01; % rolling resistance coeff
rho = 1.3; % air density
Cv = 0.32; % aero resistance coeff
Area = 2.4; % frontal area
g = 9.81; % gravitational acceleration
tauActN = 0.1;
delayN = 0.2;
windAveN = 12/3.6;
FMax = 4000; % saturating force

%% Mass uncertainty

n = 5; % change this for more resolution (approximately n*25 simulations required)
m = [1410 : 141/n : 2115]'; % max change 50%
mChange = 100*(m - mN)/mN;

%% Comfort-based reference speed

% I chose these values from the scenarios
% rmsAB = [0.315; 0.8; 1.6]; vRefAB = [40; 100; 120]/3.6;
% rmsC = [0.315; 0.8; 1.6]; vRefC = [30; 40; 50]/3.6;
% rmsD = [0.315; 0.8; 1.6]; vRefD = [30; 35; 40]/3.6;
% rmsE = [0.315; 0.8; 1.6]; vRefE = [30; 30; 30]/3.6;

updateDistance = 1000; % we update the vRef after this distance (1 km)
vRefMax = 40; % the maximum speed of the car

%% Gain-scheduled LQR controller 
mGrid = m;
v0Grid = 0 : 3 : 30;
% v0Grid = v0N;
R = 0.005;
% Cz = 100*eye(2);
% Dz = [sqrt(0.005); sqrt(0.005)];
F = zeros(length(mGrid), length(v0Grid));
H = zeros(length(mGrid), length(v0Grid));
C = [1];

for i = 1 : length(mGrid)
    for j = 1:length(v0Grid)
        % Linearized system
        A = [-rho*Area*Cv*v0Grid(j)/mGrid(i)];
        B = [1/mGrid(i)];
        % Extended system
        Ae = [A 0; -C 0];
        Be = [B; 0];
        % LQR
%         Q2 = [10000 0; 0 10000*m(i)/mN];
        Q = [10000 0; 0 10000*(m(i)/mN + v0Grid(j)/v0N)/2];
        K(: , :, i, j) = lqr(Ae, Be, Q, R);
%         [K2, lambda2, Q2, Y2] = LQRLMI(Ae2, Be2, Cz, Dz, [v0Grid(j) 0]', 1, 1, 1, 1);
        F(i, j) = K(: , :, i, j)*[1; 0];
        H(i, j) = K(: , :, i, j)*[0; 1];
    end
end

%% Car-human interaction
n_c = 0.01; % the road spatial cut-off frequency 
n_0 = 0.1;  % the standard spatial frequency
a1 = 2*pi*n_c; % the road spatial cut-off angular frequency
Gd = [64; 256; 1024; 4094]*1e-6;
b1 = 2*pi*n_0*sqrt(Gd);

% Vertical dynamics
%%% Linear front vehicle parameters ()
ms  = 315;    % [kg]*4 assuming perfect symmetry +driver
mus = 37.5;   % [kg]
k   = 29500;  % [N/m]
c   = 1500;   % [N/m/s]
kt  = 208000; % [N/m]
actuator = 208; % we'll experiment

%  x = [zs zs_p zus zus_p] 
%  u = [zr u]
%  y = [zs_pp zs_p zs zus_p zus zdef_p zdef]
Aver = [0 1 0 0; [-k -c k c]/ms; 0 0 0 1; [k c -k-kt -c]/mus];
Bver = [0 0; [0 -actuator]/ms; 0 0; [kt actuator]/mus]; 
Cver = [Aver(2,:); 0 1 0 0; 1 0 0 0; 0 0 0 1; 0 0 1 0; 0 1 0 -1; 1 0 -1 0];
Dver = [Bver(2,:); 0 0 ; 0 0; 0 0; 0 0; 0 0; 0 0];

% Filter 
% a = [1 80 2264 7172 21196];
% b = [81.89 796.6 1937 0.1446];
% w = logspace(-1, 4);
s = tf('s');
WISO_2631 = (81.89*s^3 + 796.6*s^2 + 1937*s + 0.1446)/ (s^4 + 80.00*s^3 + 2264*s^2 + 7172*s + 21196);
% h = freqs(WISO_2631,1,w);
% mag = abs(h);
% figure();
% loglog(w,mag);
% grid on;
% xlabel('Frequency (rad/s)');
% ylabel('Magnitude');

%% Display

% sim('ComfortBased_Control_Sim.slx');
% distanceTraveled = ans.results.signals(3).values;
% comfortvRef =  ans.results.signals(1).values(:, 1);
% comfortSpeed =  ans.results.signals(1).values(:, 2);
% force = ans.results.signals(2).values;
% 
% figure('Name', 'Comfort-based Speed Control Results');
% subplot(211);
% plot(distanceTraveled, comfortvRef, 'b', distanceTraveled, comfortSpeed, 'r', 'LineWidth', 4);
% grid on;
% set(gca, 'FontSize', 20); 
% xlim([1 distanceTraveled(end)]);
% xlabel('Distance traveled (km)', 'FontSize', 20);
% ylabel('Speeds (m/s)', 'FontSize', 20);
% legend('Reference speed', 'Real speed', 'FontSize', 20, 'Location', 'northeast');
% title('Comfort-oriented Longitudinal Speed Control Results - Speeds', 'FontSize', 20);
% subplot(212);
% plot(distanceTraveled, force, 'r', 'LineWidth', 4);
% grid on;
% set(gca, 'FontSize', 20); 
% xlim([1 distanceTraveled(end)]);
% xlabel('Distance traveled (km)', 'FontSize', 20);
% ylabel('Control force (N)', 'FontSize', 20);
% title('Comfort-oriented Longitudinal Speed Control Results - Control force', 'FontSize', 20);

% RMS
% sim('ComfortBased_Control_Sim.slx');
% distanceTraveled = ans.results.signals(3).values;
% fs = 200;
% window = length(out.acceleration.signals.values);
% t = (0:1/fs:(window-1)/fs);
% 
% for i = 2 : window;
%     out.acceleration.signals.values(i) = out.acceleration.signals.values(i)^2;
%     a_rms(i) = trapz(t(1:i),out.acceleration.signals.values(1:i));
%     a_rms(i) = (a_rms(i)/t(i))^0.5;
% end
% 
% figure('Name', 'RMS level');
% plot(t, a_rms, 'r', 'LineWidth', 4);
% grid on;
% set(gca, 'FontSize', 20); 
% xlim([1 t(end)]);
% xlabel('Time (s)', 'FontSize', 20);
% ylabel('Speeds (m/s)', 'FontSize', 20);
% legend('Real RMS', 'FontSize', 20, 'Location', 'northeast');
% title('Comfort-oriented Longitudinal Speed Control Results - RMS', 'FontSize', 20);

%% Robustness analysis

% mRMSE = zeros(length(m), 1);
% 
% for i = 1 : length(m)
%     mi = m(i);
%     sim('ComfortBased_Control_Sim.slx');
%     mRMSE(i) = rms(ans.results.signals(1).values(:, 1) - ans.results.signals(1).values(:, 2));
% end
% 
% mRSMEChange = 100*abs(mRMSE - mRMSE(1))/mRMSE(1); % change in mass RMSE
% 
% mIR = mRSMEChange./mChange; % impact rate
% 
% figure('Name', 'Car mass robustness analysis');
% plot(mChange, mRSMEChange, 'r', 'LineWidth', 4);
% grid on;
% xlim([0 50]);
% ylim([0 20]);
% set(gca, 'FontSize', 20); 
% xlabel('Change in car mass [%]', 'FontSize', 20);
% ylabel('Change in RMSE [%]', 'FontSize', 20);
% title('Car mass robustness analysis', 'FontSize', 20);