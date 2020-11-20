%% Speed control with LQR + integral action - Tran Gia Quoc Bao

%% Default commands
close all;
clear all;
clc;

%% Values
%Car
m = 1400; 

% Frictions
Cr = 0.01;
rho = 1.3;
Cv = 0.32;
Area = 2.4;
g = 9.81;

%% Actuator
% It has order 1
tauAct = 0.1; % settling time = 0.3 s
sysAct = tf(1, [tauAct 1]);
delay = 0.2;

%% Road profile
load data; % the altitude is measured after each 30m of movement
time = altitude(1, 1 : end - 1)';
slope = zeros(length(altitude) - 1, 1);
for i = 1 : length(altitude) - 1
    slope(i) = (altitude(2, i + 1) - altitude(2, i))/30;
end
% n = 10;
% v0 = 22;
% s = 40*ones(n, 1); 
% slope = (pi/180)*(10*randn(n, 1)); % segments of Gaussian random slopes from -10 to 10 degrees
% Fl1 = 3000;
% Fd10 = 1000;
% Fdr = zeros(n, 1);
% vRef = zeros(n, 1);
% 
% vRef(1) = sqrt((v0^2 + (2/m)*s(1)*(Fl1 - Fd10 - m*g*sin(slope(1))))/(1 + s(1)*rho*Cv*Area/m));
% trip(1) = s(1); % to find the total distance traveled
% attitude(1) = s(1)*sin(slope(1)); % to plot the attitude
% 
% for i = 2 : n
%     vRef(i) = sqrt(((vRef(i-1))^2 - 2*s(i)*g*sin(slope(i)))/(1 + s(1)*rho*Cv*Area/m));
% end 
% 
% time(1) = s(1)/vRef(1);
% for i = 2 : n
%     time(i) = time(i-1) + s(i)/vRef(i);
% end 

lookaheadspeed = lookaheadspeed(2,:);
lookaheadspeed = lookaheadspeed(1:26644)/3.6; % take some values and change unit to m/s
% Workspace data
slopeTime = [time slope];
vRef = [time lookaheadspeed'];
% vRefTime = [time' vRef];


%% LQR controller
v0 = 22;
theta0 = 0;
u0 = 0.5*rho*Area*Cv*v0^2 + m*g*Cr;
Fd0 = m*g*Cr + 0.5*rho*Cv*Area*v0^2 + m*g*theta0;

A = [-rho*Area*Cv*v0/m];
B = [1/m];
C = [1];
D = 0;
sys = ss(A, B, C, D, 'Statename', {'dv'}, 'Inputname', {'du'}, 'Outputname', {'dv'});
% The system is both controllable and observable. So we can build an LQR
% controller for it.

% Extended system
Ae = [A 0; -C 0];
Be = [B; 0]; % we do not include the noise theta
Ce = [C 0];
sysExt = ss(Ae, Be, Ce, D);
% Controllability
controllabilityExt = rank(ctrb(sysExt)); 
% CONTROLLABLE as rank = 2 

% LQR controller
Q = [10000 0; 0 10000];
R = 0.01;
Fe = lqr(Ae, Be, Q, R);
F = Fe(1);
H = Fe(2);