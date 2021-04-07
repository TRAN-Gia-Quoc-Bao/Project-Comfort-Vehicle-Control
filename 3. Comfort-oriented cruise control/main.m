% LMI-based LQR cruise control

close all; clear all; clc;

% Parameters
v0 = 11; Cr = 0.01; Da = 1.3; Cv = 0.32; S = 2.4; g = 9.81; tauAct = 0; delay = 0; windAve = 12/3.6; FMax = 5000;

% Parameter-dependent representation (rho1 = 1/m, rho2 = v)
%C = [1 0]; D = 0;
rho1Min = 1/1680; rho1Max = 1/1400; rho2Min = 0; rho2Max = 35;

A1 = [-0.5*Cv*Da*S*rho1Min*rho2Min 0; -1 0]; B1 = [rho1Min; 0];
A2 = [-0.5*Cv*Da*S*rho1Min*rho2Max 0; -1 0]; B2 = [rho1Min; 0];
A3 = [-0.5*Cv*Da*S*rho1Max*rho2Min 0; -1 0]; B3 = [rho1Max; 0];
A4 = [-0.5*Cv*Da*S*rho1Max*rho2Max 0; -1 0]; B4 = [rho1Max; 0];

% Parameter independent B, new state = [x1 x2 xf]^T
tau = 0.001; Af = -1/tau; Bf = 1/tau; Cf = 1; Df = 0;
Bi = [0; 0; Bf]; %Ci = [C D*Cf]; Di = 0;
Ai1 = [A1 B1*Cf; zeros(1, 2) Af];
Ai2 = [A2 B2*Cf; zeros(1, 2) Af];
Ai3 = [A3 B3*Cf; zeros(1, 2) Af]; 
Ai4 = [A4 B4*Cf; zeros(1, 2) Af]; 

% Controller
x0 = [v0; 0; 0]; % initial condition
Cz = diag([1 10 0]); Dz = [2e-2; 0; 0]; % weight on the integral of the tracking error
[K1, K2, K3, K4, lambda, Q] = LQRLMI_cvx(Ai1, Ai2, Ai3, Ai4, Bi, Cz, Dz, x0);

%% Simulation
tsim = 120; % in MDPI paper: 54s
Ts = 0.005;
my_opt = simset('InitialStep', 1e-3, 'OutputVariables', 'txy');
resultTracking = sim('longitudinalControl2', [0, tsim], my_opt);
% error = resultTrackingHinf.cruise.signals(1).values(:, 1) - resultTrackingHinf.cruise.signals(1).values(:, 2);
% plot(resultTrackingHinf.cruise.time, error, 'r', 'LineWidth', 3);
speed = resultTracking.cruise.signals(1).values(:, 2);
time = resultTracking.cruise.time;
roughness = resultTracking.cruise.signals(5).values;
save('speed_unc.mat', 'time', 'speed', 'roughness');

%% Plot
figure(1); 
plot(resultTracking.cruise.signals(3).values, resultTracking.cruise.signals(1).values(:, 1), 'b',resultTracking.cruise.signals(3).values, resultTracking.cruise.signals(1).values(:, 2), 'r', 'LineWidth', 3);
% title('Vehicle cruise control using LPV-based $H_{\infty}$ with road slope compensation', 'Interpreter', 'latex');
% title('Vehicle cruise control using LPV $\mathcal{H}_2$ with road slope compensation', 'Interpreter', 'latex');
ylabel('Speed [m/s]', 'Interpreter', 'latex');
xlabel('Road [m]', 'Interpreter', 'latex');
xlim([0 resultTracking.cruise.signals(3).values(end)]);
ylim([0 33]);
% legend('Reference speed', 'Vehicle speed', 'Location', 'northwest', 'Interpreter', 'latex');
legend('Reference speed', 'Vehicle speed', 'Location', 'southeast', 'Interpreter', 'latex');
grid on;
set(gca,'fontsize', 28);
%%
figure(2); 
plot(resultTracking.cruise.signals(3).values, resultTracking.cruise.signals(2).values, 'b', 'LineWidth', 2);
% title('Vehicle cruise control using LPV-based $H_{\infty}$ with road slope compensation', 'Interpreter', 'latex');
% title('Vehicle cruise control using LPV $\mathcal{H}_2$ with road slope compensation', 'Interpreter', 'latex');
ylabel('Force [N]', 'Interpreter', 'latex');
xlabel('Road [m]', 'Interpreter', 'latex');
xlim([0 resultTracking.cruise.signals(3).values(end)]);
% ylim([-3200 4000]);
% ylim([-2100 3000]); % if H-inf
% legend('Control force', 'Location', 'southeast', 'Interpreter', 'latex');
grid on;
set(gca,'fontsize', 28);
%%
figure(3); 
plot(resultTracking.cruise.signals(3).values, resultTracking.cruise.signals(1).values(:, 1) - resultTracking.cruise.signals(1).values(:, 2), 'k', 'LineWidth', 2);
% title('Vehicle cruise control using LPV-based $H_{\infty}$ with road slope compensation', 'Interpreter', 'latex');
% title('Vehicle cruise control using LPV $\mathcal{H}_2$ with road slope compensation', 'Interpreter', 'latex');
ylabel('Speed tracking error [m/s]', 'Interpreter', 'latex');
xlabel('Road [m]', 'Interpreter', 'latex');
xlim([0 resultTracking.cruise.signals(3).values(end)]);
ylim([-7 7]);
% ylim([-2100 3000]); % if H-inf
% legend('Control force', 'Location', 'southeast', 'Interpreter', 'latex');
grid on;
set(gca,'fontsize', 28);
%%
figure(4); 
plot(resultTracking.cruise.signals(3).values, resultTracking.cruise.signals(4).values, 'b', 'LineWidth', 2);
% title('Vehicle cruise control using LPV-based $H_{\infty}$ with road slope compensation', 'Interpreter', 'latex');
% title('Vehicle cruise control using LPV $\mathcal{H}_2$ with road slope compensation', 'Interpreter', 'latex');
ylabel('RMS acceleration [m/s$^2$]', 'Interpreter', 'latex');
xlabel('Road [m]', 'Interpreter', 'latex');
xlim([0 resultTracking.cruise.signals(3).values(end)]);
% ylim([-7 7]);
% ylim([-2100 3000]); % if H-inf
% legend('Control force', 'Location', 'southeast', 'Interpreter', 'latex');
grid on;
set(gca,'fontsize', 28);
%%
figure(5); 
plot(resultTracking.cruise.signals(3).values, resultTracking.cruise.signals(5).values, 'b', 'LineWidth', 2);
% title('Vehicle cruise control using LPV-based $H_{\infty}$ with road slope compensation', 'Interpreter', 'latex');
% title('Vehicle cruise control using LPV $\mathcal{H}_2$ with road slope compensation', 'Interpreter', 'latex');
ylabel('Road roughness [m]', 'Interpreter', 'latex');
xlabel('Road [m]', 'Interpreter', 'latex');
xlim([0 resultTracking.cruise.signals(3).values(end)]);
% ylim([-7 7]);
% ylim([-2100 3000]); % if H-inf
% legend('Control force', 'Location', 'southeast', 'Interpreter', 'latex');
grid on;
set(gca,'fontsize', 28);