%% LQR + gain-scheduling - TRAN Gia Quoc Bao

%% Default commands
close all;
clear all;
clc;

%% Values
mNominal = 1400;
% Frictions
Cr = 0.01;
rho = 1.3;
Cv = 0.32;
Area = 2.4;
g = 9.81;
% Actuator
tauAct = 0.1; % settling time = 0.3 s
sysAct = tf(1, [tauAct 1]);
delay = 0.2;
timeCom = 0.05;

%% Road profile
load data; % the altitude is measured after each 30m of movement
time = altitude(1, 1 : end - 1)';
slope = zeros(length(altitude) - 1, 1);
for i = 1 : length(altitude) - 1
    slope(i) = (altitude(2, i + 1) - altitude(2, i))/30;
end
lookaheadspeed = lookaheadspeed(2,:);
lookaheadspeed = lookaheadspeed(1:26644)/3.6; % take some values and change unit to m/s
% Workspace data
slopeTime = [time slope];
vRef = [time lookaheadspeed'];

%% Parameter grid
m = 1400 : 100 : 2100; % 1st varying parameter = car's mass (kg), dimension  = 8
v0 = 16 : 2 : 36; % 2nd varying parameter = car's reference speed (m/s), dimension = 11
[mGrid, v0Grid] = ndgrid(m, v0);

% Linearized system
C = [1];
D = 0;
% Extended system
Ce = [C 0];
% LQR
Q = [10000 0; 0 10000];
R = 0.001;
F = zeros(length(m), length(v0));
H = zeros(length(m), length(v0));
u0 = zeros(length(m), length(v0));

for i = 1:length(m)
    for j = 1:length(v0)
        % Linearized system
        A = [-rho*Area*Cv*v0(j)/m(i)];
        B = [1/m(i)];
        sys(:, :, i, j) = ss(A, B, C, D);
        % Extended system
        Ae = [A 0; -C 0];
        Be = [B; 0];
        sysExt(:, :, i, j) = ss(Ae, Be, Ce, D);
        % LQR
        Fe(: , :, i, j) = lqr(Ae, Be, Q, R);
        F(i, j) = Fe(: , :, i, j)*[1; 0];
        H(i, j) = Fe(: , :, i, j)*[0; 1];
        u0(i, j) = 0.5*rho*Area*Cv*(v0(j))^2 + m(i)*g*Cr;
    end
end

% Assign systems & gain sets to grid
sys.SamplingGrid = struct('m', mGrid, 'v0', v0Grid);
sysExt.SamplingGrid = struct('m', mGrid, 'v0', v0Grid);
Fet = struct('Fe', Fe);
Fet.SamplingGrid = struct('m', mGrid, 'v0', v0Grid);