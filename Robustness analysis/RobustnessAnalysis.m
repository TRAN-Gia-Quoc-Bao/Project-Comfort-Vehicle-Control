%% Uncertainty analysis - TRAN Gia Quoc Bao

%% Default commands
close all;
clear all;
clc;

%% Values
% Nominal values
mN = 1400; % mass
v0N = 20;
smN = 1; % nominal slope magnitude
CrN = 0.01; % rolling resistance coeff
rho = 1.3; % air density
Cv = 0.32; % aero resistance coeff
Area = 2.4; % frontal area
g = 9.81; % gravitational acceleration
tauActN = 0.1;
delayN = 0.2;
windAveN = 12/3.6;
FMax = 4000;

%% Uncertainties
n = 1; % change this for more resolution (but approximately n*150 simulations required so be careful)
% 1. Mass
m = [1400 : 30/n : 2100]'; % max change 50%
mChange = 100*(m - mN)/mN;
% 2. Slope magnitude
sm = [1 : 0.1/n : 1.5]';
smChange = 100*(sm - smN)/smN;
% 3. Rolling resistance coeff
Cr = [0.01 : 0.0002/n : 0.015]';
CrChange = 100*(Cr - CrN)/CrN; 
% 4. Actuator dynamics
tauAct = [0.1 : 0.002/n : 0.15]'; % accelerator dynamics, settling time = 3*tauAct
tauActChange = 100*(tauAct - tauActN)/tauActN; 
% 5. Transmission delay
delay = [0.2 : 0.005/n : 0.3]';
delayChange = 100*(delay - delayN)/delayN;
% 6. Wind speed 
windAve = [12/3.6 : (0.25/3.6)/n : 18/3.6]';
windAveChange = 100*(windAve - windAveN)/windAveN;

%% Static LQR controller
A1 = [-rho*Area*Cv*v0N/mN];
B1 = [1/mN];
C1 = [1];
D1 = 0;
% Extended system
Ae1 = [A1 0; -C1 0];
Be1 = [B1; 0]; % we do not include the noise theta
Ce1 = [C1 0];

Q1 = [10000 0; 0 10000];
R1 = 0.005;
K1 = lqr(Ae1, Be1, Q1, R1);
F1 = K1(1);
H1 = K1(2);

%% Gain-scheduled LQR controller 
mGrid = m;
v0Grid = 0 : 3 : 30;
% v0Grid = v0N;
R2 = 0.005;
Cz = 100*eye(2);
Dz = [sqrt(0.005); sqrt(0.005)];
F2 = zeros(length(mGrid), length(v0Grid));
H2 = zeros(length(mGrid), length(v0Grid));
C2 = [1];

for i = 1 : length(mGrid)
    for j = 1:length(v0Grid)
        % Linearized system
        A2 = [-rho*Area*Cv*v0Grid(j)/mGrid(i)];
        B2 = [1/mGrid(i)];
        % Extended system
        Ae2 = [A2 0; -C2 0];
        Be2 = [B2; 0];
        % LQR
%         Q2 = [10000 0; 0 10000*m(i)/mN];
        Q2 = [10000 0; 0 10000*(m(i)/mN + v0Grid(j)/v0N)/2];
        K2(: , :, i, j) = lqr(Ae2, Be2, Q2, R2);
%         [K2, lambda2, Q2, Y2] = LQRLMI(Ae2, Be2, Cz, Dz, [v0Grid(j) 0]', 1, 1, 1, 1);
        F2(i, j) = K2(: , :, i, j)*[1; 0];
        H2(i, j) = K2(: , :, i, j)*[0; 1];
    end
end

%% Visualize the grid
% For illustration
% [X, Y] = meshgrid([0 : 0.5 : 10], [0 : 1 : 20]);
% figure();
% Z = (Y/2).*sin(X/2) - (X/2).*cos(Y/2);
% surf(X, Y, Z);
% set(gca, 'FontSize', 20);
% xlabel('First parameter', 'FontSize', 20);
% ylabel('Second parameter', 'FontSize', 20);
% zlabel('Scheduled gain', 'FontSize', 20);
% title('Illustration of grid-based gain-scheduling', 'FontSize', 20);
% 
% [X, Y] = meshgrid(mGrid', v0Grid');
% 
% figure();
% surf(X, Y, F2');
% set(gca, 'FontSize', 20);
% xlabel('Car mass [kg]', 'FontSize', 20);
% ylabel('Reference speed [m/s]', 'FontSize', 20);
% zlabel('State gain','FontSize', 20);
% title('Grid-based state gain', 'FontSize', 20);
% 
% figure();
% surf(X, Y, H2');
% set(gca, 'FontSize', 20);
% xlabel('Car mass [kg]', 'FontSize', 20);
% ylabel('Reference speed [m/s]', 'FontSize', 20);
% zlabel('Integral gain', 'FontSize', 20);
% title('Grid-based integral gain', 'FontSize', 20);

%% System analysis with uncertainty
% mUnc = ureal('m_unc', mN, 'Percentage', [-1, 50]);
% 
% % OPEN-LOOP
% AUnc = [-rho*Area*Cv*v0N/mUnc];
% BUnc = [1/mUnc];
% CUnc = [1];
% DUnc = 0;
% 
% modelUnc = ss(AUnc, BUnc, CUnc, DUnc);
% 
% % Poles & zeros
% figure();
% pzmap(modelUnc);
% % The system's pole is the A matrix itself and it's always negative. The
% % larger the mass, the closer the pole to the imaginary axis, and the
% % slower the system. This is coherent with the fact that the system's
% % inertia increases with the mass.
% 
% figure();
% step(modelUnc);
% grid on;
% % The open-loop system's order is 1 so never is there an overshoot.
% 
% figure();
% bode(modelUnc);
% grid on;
% % The curves of 1st-order system. No significant change.

% CLOSE-LOOP
% Cm = [1];
% Dm = 0;
% for i = 1 : length(m)
%     for j = 1:length(v0Grid)
%         Am = [-rho*Area*Cv*v0Grid(j)/m(i) 0; -Cm 0];
%         Bm = [1/m(i); 0];
%         
%         AStatic = Am - Bm*K1;
%         sysStatic = ss(AStatic, [1; 1], [1 0], 0);
%         [GmStatic(i, j), PmStatic(i, j), WcgStatic(i, j), WcpStatic(i, j)] = margin(sysStatic);
%         
%         AGS = Am - Bm*K2(: , :, i, j);
%         sysGS = ss(AGS, [1; 1], [1 0], 0);
%         [GmGS(i, j), PmGS(i, j), WcgGS(i, j), WcpGS(i, j)] = margin(sysGS);
%     end
% end
% 
% [X, Y] = meshgrid(mGrid', v0Grid');
% figure();
% subplot(221);
% surf(X, Y, GmStatic');
% title('Gain margin, static integral LQR');
% subplot(222);
% surf(X, Y, GmGS');
% title('Gain margin, GS integral LQR');
% subplot(223);
% surf(X, Y, PmStatic');
% title('Phase margin, static integral LQR');
% subplot(224);
% surf(X, Y, PmGS');
% title('Phase margin, GS integral LQR');

% Inf is because they are stable systems with no output.

%% Balasz's road profile
% load data; % the altitude is measured after each 1m of movement
% lookaheadspeed = lookaheadspeed(2,:);
% lookaheadspeed = lookaheadspeed(1:26644)/3.6; % take some values and change unit to m/s
% 
% timeIncrement = altitude(1, 1 : end - 1)'./lookaheadspeed';
% timeSlope = zeros(length(altitude) - 1, 1); % time for the slope
% timeSlope(1) = timeIncrement(1);
% for i = 2 : length(altitude) - 1
%     timeSlope(i) = timeSlope(i - 1) + timeIncrement(i);
% end
% 
% timeSpeed = zeros(length(altitude) - 1, 1); % time for the reference speed
% timeSpeed(1) = 30/lookaheadspeed(1);
% for i = 2 : length(altitude) - 1
%     timeSpeed(i) = timeSpeed(i - 1) + 30/lookaheadspeed(i);
% end
% 
% slope = zeros(length(altitude) - 1, 1);
% for i = 1 : length(altitude) - 1
%     slope(i) = (altitude(2, i + 1) - altitude(2, i))/1;
% end
% 
% % Workspace data (it takes 33928s for the simulation)
% slopeTime = [timeSlope/10000 slope];
% vRef = [timeSpeed lookaheadspeed'];

% figure();
% subplot(211);
% plot(timeSpeed, lookaheadspeed, 'LineWidth', 4);
% grid on;
% set(gca, 'FontSize', 20);
% xlabel('Time (seconds)', 'FontSize', 20);
% ylabel('Speed (m/s)', 'FontSize', 20);
% title('Real speeds', 'FontSize', 20);
% subplot(212);
% plot(timeSlope, slope, 'LineWidth', 4);
% grid on;
% set(gca, 'FontSize', 20);
% xlabel('Time (seconds)', 'FontSize', 20);
% ylabel('Slope (rad)', 'FontSize', 20);
% title('Real slopes', 'FontSize', 20);

%% Calculate Impact Rate (mass, sd, Cr, tauAct, delay, wind)
% Definition: IR = (% change in RMSE)/(% change in uncertainty)
%% Nominal case
mi = mN; smi = smN; Cri = CrN; tauActi = tauActN; delayi = delayN; windAvei = windAveN;
sim('CarUncertaintyAnalysis.slx');
RMSEN = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2)); 
RMSEN_GS = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3)); 
RMSEN_GS_Com = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4)); 

%% 1. Mass 
% Other uncertainties go to nominal
smi = smN; Cri = CrN; tauActi = tauActN; delayi = delayN; windAvei = windAveN;

mRMSE = zeros(length(m), 1);
mRMSE_GS = zeros(length(m), 1);
mRMSE_GS_Com = zeros(length(m), 1);

for i = 1 : length(m)
    mi = m(i);
    sim('CarUncertaintyAnalysis.slx');
    mRMSE(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2));
    mRMSE_GS(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3));
    mRMSE_GS_Com(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4));
end

mRSMEChange = 100*abs(mRMSE - RMSEN)/RMSEN;
mRSMEChange_GS = 100*abs(mRMSE_GS - RMSEN_GS)/RMSEN_GS;
mRSMEChange_GS_Com = 100*abs(mRMSE_GS_Com - RMSEN_GS_Com)/RMSEN_GS_Com;

mIR = mRSMEChange./mChange;
mIR_GS = mRSMEChange_GS./mChange;
mIR_GS_Com = mRSMEChange_GS_Com./mChange;

%% 2. Slope magnitude
% Other uncertainties go to nominal
mi = mN; Cri = CrN; tauActi = tauActN; delayi = delayN; windAvei = windAveN;

smRMSE = zeros(length(sm), 1);
smRMSE_GS = zeros(length(sm), 1);
smRMSE_GS_Com = zeros(length(sm), 1);

for i = 1 : length(sm)
    smi = sm(i);
    sim('CarUncertaintyAnalysis.slx');
    smRMSE(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2));
    smRMSE_GS(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3));
    smRMSE_GS_Com(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4));
end

smRSMEChange = 100*abs(smRMSE - RMSEN)/RMSEN;
smRSMEChange_GS = 100*abs(smRMSE_GS - RMSEN_GS)/RMSEN_GS;
smRSMEChange_GS_Com = 100*abs(smRMSE_GS_Com - RMSEN_GS_Com)/RMSEN_GS_Com;

smIR = smRSMEChange./smChange;
smIR_GS = smRSMEChange_GS./smChange;
smIR_GS_Com = smRSMEChange_GS_Com./smChange;

%% 3. Rolling resistance coeff
% Other uncertainties go to nominal
mi = mN; smi = smN; tauActi = tauActN; delayi = delayN; windAvei = windAveN;

CrRMSE = zeros(length(Cr), 1);
CrRMSE_GS = zeros(length(Cr), 1);
CrRMSE_GS_Com = zeros(length(Cr), 1);

for i = 1 : length(Cr)
    Cri = Cr(i);
    sim('CarUncertaintyAnalysis.slx');
    CrRMSE(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2));
    CrRMSE_GS(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3));
    CrRMSE_GS_Com(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4));
end

CrRSMEChange = 100*abs(CrRMSE - RMSEN)/RMSEN;
CrRSMEChange_GS = 100*abs(CrRMSE_GS - RMSEN_GS)/RMSEN_GS;
CrRSMEChange_GS_Com = 100*abs(CrRMSE_GS_Com - RMSEN_GS_Com)/RMSEN_GS_Com;

CrIR = CrRSMEChange./CrChange;
CrIR_GS = CrRSMEChange_GS./CrChange;
CrIR_GS_Com = CrRSMEChange_GS_Com./CrChange;

%% 4. Actuator dynamics
% Other uncertainties go to nominal
mi = mN; smi = smN; Cri = CrN; delayi = delayN; windAvei = windAveN;

tauActRMSE = zeros(length(tauAct), 1);
tauActRMSE_GS = zeros(length(tauAct), 1);
tauActRMSE_GS_Com = zeros(length(tauAct), 1);

for i = 1 : length(tauAct)
    tauActi = tauAct(i);
    sim('CarUncertaintyAnalysis.slx');
    tauActRMSE(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2));
    tauActRMSE_GS(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3));
    tauActRMSE_GS_Com(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4));
end

tauActRSMEChange = 100*abs(tauActRMSE - RMSEN)/RMSEN;
tauActRSMEChange_GS = 100*abs(tauActRMSE_GS - RMSEN_GS)/RMSEN_GS;
tauActRSMEChange_GS_Com = 100*abs(tauActRMSE_GS_Com - RMSEN_GS_Com)/RMSEN_GS_Com;

tauActIR = tauActRSMEChange./tauActChange;
tauActIR_GS = tauActRSMEChange_GS./tauActChange;
tauActIR_GS_Com = tauActRSMEChange_GS_Com./tauActChange;

%% 5. Transmission delay
% Other uncertainties go to nominal
mi = mN; smi = smN; Cri = CrN; tauActi = tauActN; windAvei = windAveN;

delayRMSE = zeros(length(delay), 1);
delayRMSE_GS = zeros(length(delay), 1);
delayRMSE_GS_Com = zeros(length(delay), 1);

for i = 1 : length(delay)
    delayi = delay(i);
    sim('CarUncertaintyAnalysis.slx');
    delayRMSE(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2));
    delayRMSE_GS(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3));
    delayRMSE_GS_Com(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4));
end

delayRSMEChange = 100*abs(delayRMSE - RMSEN)/RMSEN;
delayRSMEChange_GS = 100*abs(delayRMSE_GS - RMSEN_GS)/RMSEN_GS;
delayRSMEChange_GS_Com = 100*abs(delayRMSE_GS_Com - RMSEN_GS_Com)/RMSEN_GS_Com;

delayIR = delayRSMEChange./delayChange;
delayIR_GS = delayRSMEChange_GS./delayChange;
delayIR_GS_Com = delayRSMEChange_GS_Com./delayChange;

%% 6. Wind speed
% Other uncertainties go to nominal
mi = mN; smi = smN; Cri = CrN; tauActi = tauActN; delayi = delayN; windAvei = windAveN;

windAveRMSE = zeros(length(windAve), 1);
windAveRMSE_GS = zeros(length(windAve), 1);
windAveRMSE_GS_Com = zeros(length(windAve), 1);

for i = 1 : length(windAve)
    windAvei = windAve(i);
    sim('CarUncertaintyAnalysis.slx');
    windAveRMSE(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 2));
    windAveRMSE_GS(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 3));
    windAveRMSE_GS_Com(i) = rms(ans.Speed.signals.values(:, 1) - ans.Speed.signals.values(:, 4));
end

windAveRSMEChange = 100*abs(windAveRMSE - RMSEN)/RMSEN;
windAveRSMEChange_GS = 100*abs(windAveRMSE_GS - RMSEN_GS)/RMSEN_GS;
windAveRSMEChange_GS_Com = 100*abs(windAveRMSE_GS_Com - RMSEN_GS_Com)/RMSEN_GS_Com;

windAveIR = windAveRSMEChange./windAveChange;
windAveIR_GS = windAveRSMEChange_GS./windAveChange;
windAveIR_GS_Com = windAveRSMEChange_GS_Com./windAveChange;

% Test with small reference speed then increase. Make an excel file. Make a
% simple reference speed profile & road profile.

%% Plot change in RMSE
%% 1. Mass
figure('Name', 'Robustness analysis for m');
plot(mChange, mRSMEChange, 'g', mChange, mRSMEChange_GS, 'b', mChange, mRSMEChange_GS_Com, 'r', 'LineWidth', 4);
grid on;
xlim([0 50]);
ylim([0 28]);
set(gca, 'FontSize', 20); 
legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 20, 'Location', 'northeast');
xlabel('Change in m [%]', 'FontSize', 20);
ylabel('Change in RMSE [%]', 'FontSize', 20);
title('Car mass robustness analysis', 'FontSize', 20);

%% 2. Slope magnitude
figure('Name', 'Robustness analysis for theta');
plot(smChange, smRSMEChange, 'g', smChange, smRSMEChange_GS, 'b', smChange, smRSMEChange_GS_Com, 'r', 'LineWidth', 4);
grid on;
xlim([0 50]);
ylim([0 15]);
set(gca, 'FontSize', 20); 
legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 20, 'Location', 'northeast');
xlabel('Change in theta [%]', 'FontSize', 20);
ylabel('Change in RMSE [%]', 'FontSize', 20);
title('Slope magnitude robustness analysis', 'FontSize', 20);

%% 3. Rolling resistance coeff
figure('Name', 'Robustness analysis for C_r');
plot(CrChange, CrRSMEChange, 'g', CrChange, CrRSMEChange_GS, 'b', CrChange, CrRSMEChange_GS_Com, 'r', 'LineWidth', 4);
grid on;
xlim([0 50]);
ylim([0 15]);
set(gca, 'FontSize', 20); 
legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 20, 'Location', 'northeast');
xlabel('Change in C_r [%]', 'FontSize', 20);
ylabel('Change in RMSE [%]', 'FontSize', 20);
title('Rolling resistance coefficient robustness analysis', 'FontSize', 20);

%% 4. Actuator dynamics
figure('Name', 'Robustness analysis for \tau_{act}');
plot(tauActChange, tauActRSMEChange, 'g', tauActChange, tauActRSMEChange_GS, 'b', tauActChange, tauActRSMEChange_GS_Com, 'r', 'LineWidth', 4);
grid on;
xlim([0 50]);
ylim([0 20]);
set(gca, 'FontSize', 20);
legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 20, 'Location', 'northeast');
xlabel('Change in \tau_{act} [%]', 'FontSize', 20);
ylabel('Change in RMSE [%]', 'FontSize', 20);
title('Actuator dynamics robustness analysis', 'FontSize', 20);

%% 5. Transmission delay
figure('Name', 'Robustness analysis \delta T');
plot(delayChange, delayRSMEChange, 'g', delayChange, delayRSMEChange_GS, 'b', delayChange, delayRSMEChange_GS_Com, 'r', 'LineWidth', 4);
grid on;
xlim([0 50]);
ylim([0 10]);
set(gca, 'FontSize', 20);
legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 20, 'Location', 'northeast');
xlabel('Change in \delta T [%]', 'FontSize', 20);
ylabel('Change in RMSE [%]', 'FontSize', 20);
title('Transmission delay robustness analysis', 'FontSize', 20);

%% 6. Wind speed
figure('Name', 'Robustness analysis for v_{wind}');
plot(windAveChange, windAveRSMEChange, 'g', windAveChange, windAveRSMEChange_GS, 'b', windAveChange, windAveRSMEChange_GS_Com, 'r', 'LineWidth', 4);
grid on;
xlim([0 50]);
ylim([0 16]);
set(gca, 'FontSize', 20);
legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 20, 'Location', 'northeast');
xlabel('Change in v_{wind} [%]', 'FontSize', 20);
ylabel('Change in RMSE [%]', 'FontSize', 20);
title('Average wind speed robustness analysis', 'FontSize', 20);

%% Plot IR
% 1. Mass
% figure('Name', 'Car mass uncertainty analysis');
% plot(mChange, mIR, 'g', mChange, mIR_GS, 'b', mChange, mIR_GS_Com, 'r', 'LineWidth', 2);
% grid on;
% legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 14, 'Location', 'northeast');
% xlabel('Change in car mass [%]');
% ylabel('Impact rate of car mass');
% title('Car mass uncertainty analysis');
% 
% % 2. Slope magnitude
% figure('Name', 'Slope magnitude uncertainty analysis');
% plot(smChange, smIR, 'g', smChange, smIR_GS, 'b', smChange, smIR_GS_Com, 'r', 'LineWidth', 2);
% grid on;
% legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 14, 'Location', 'northeast');
% xlabel('Change in slope magnitude [%]');
% ylabel('Impact rate of slope magnitude');
% title('Slope magnitude uncertainty analysis');
% 
% % 3. Rolling resistance coeff
% figure('Name', 'Slope standard deviation uncertainty analysis');
% plot(CrChange, CrIR, 'g', CrChange, CrIR_GS, 'b', CrChange, CrIR_GS_Com, 'r', 'LineWidth', 2);
% grid on;
% legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 14, 'Location', 'northeast');
% xlabel('Change in rolling resistance coefficient [%]');
% ylabel('Impact rate of rolling resistance coefficient');
% title('Rolling resistance coefficient uncertainty analysis');
% 
% % 4. Actuator dynamics
% figure('Name', 'Actuator dynamics uncertainty analysis');
% plot(tauActChange, tauActIR, 'g', tauActChange, tauActIR_GS, 'b', tauActChange, tauActIR_GS_Com, 'r', 'LineWidth', 2);
% grid on;
% legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 14, 'Location', 'northeast');
% xlabel('Change in actuator time constant [%]');
% ylabel('Impact rate of actuator dynamics');
% title('Actuator dynamics uncertainty analysis');
% 
% % 5. Transmission delay
% figure('Name', 'Transmission delay uncertainty analysis');
% plot(delayChange, delayIR, 'g', delayChange, delayIR_GS, 'b', delayChange, delayIR_GS_Com, 'r', 'LineWidth', 2);
% grid on;
% legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation', 'FontSize', 14, 'Location', 'northeast');
% xlabel('Change in transmission delay [%]');
% ylabel('Impact rate of transmission delay');
% title('Transmission delay uncertainty analysis');
% 
% % 6. Wind speed
% figure('Name', 'Average wind speed uncertainty analysis');
% plot(windAveChange, windAveIR, 'g', windAveChange, windAveIR_GS, 'b', windAveChange, windAveIR_GS_Com, 'r', 'LineWidth', 2);
% grid on;
% legend('Static LQR with integral action', 'Gain-scheduled LQR with integral action', 'Gain-scheduled LQR with integral action and compensation','FontSize', 14, 'Location', 'northeast');
% xlabel('Change in average wind speed [%]');
% ylabel('Impact rate of average wind speed');
% title('Average wind speed uncertainty analysis');