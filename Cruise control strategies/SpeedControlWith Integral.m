%% Speed control with integral action - Tran Gia Quoc Bao
% Equations from "Control and Dynamical Systems" - CalTech

%% Default commands
close all;
clear all;
clc;

%% Values
%Car
m = 2037; % car + 2 passengers
an = 12; % alpha_n: 40 25, 16, 12, 10
Tm = 190; % maximum torque
wm = 420; % maximum engine speed
beta = 0.4;
w = 0 : 10 : 700;
vRef = 0 : 1 : 70;

% Frictions
Cr = 0.01;
rho = 1.3;
Cv = 0.32;
Area = 2.4;
g = 9.81;

%% Torque curve
T = Tm*(1 - beta*(w/wm - 1).^2);

figure();
plot(w, T);
grid on;
xlabel('Engine speed (rad/s)');
ylabel('Engine torque (Nm)');
title('Torque curve');
axis([0 700 0 200]);
% It looks like in the lecture notes

%% Forces
theta0 = 0;
Fd = m*g*Cr + 0.5*rho*Cv*Area*vRef.^2 + m*g*theta0;
u = [0.2 0.5 1];

figure();
plot(vRef, Fd);
hold on
for i = 1:3
    plot(vRef, u(i)*an*T, '--');
    hold on;
end
legend('Fd', 'F, u = 0.2', 'F, u = 0.5', 'F, u = 1');
grid on;
xlabel('Car speed (m/s)');
ylabel('Forces (N)');
title('Forces on the car at the equilibrium point');
axis([0 70 0 2500]);
% The curves look like in the lecture notes

%% System & analysis
% State space
v0 = 22;
u0 = 0.5*rho*Area*Cv*v0^2 + m*g*Cr;
Fd0 = m*g*Cr + 0.5*rho*Cv*Area*v0^2 + m*g*theta0;
A = [-rho*Area*Cv*v0/m];
B = [1/m];
C = [1];
D = 0;
sys = ss(A, B, C, D, 'Statename', {'dv'}, 'Inputname', {'du'}, 'Outputname', {'dv'});

% Stability
eig(A); % STABLE (1 eigen value = 0, the other one negative)

% Controllability
controllability = rank(ctrb(sys)); 
% CONTROLLABLE as rank = 1 

% Observability
observability = rank(obsv(sys)); 
% OBSERVABLE (rank = 1) 

% Frequency analysis
figure();
bodemag(sys);
grid on;
% The system is badly disturbed by the noise theta

% Unforced response
figure();
initial(sys, [0.1]);
grid on;
% It converges as the system is stable but very slowly as a/m is too small

% Force response
figure();
step(sys);
grid on;
% It has no visible change as a/m is too small and the change is too slow to see 

%% Actuator
% It has order 1
tauAct = 0.1;
sysAct = tf(1, [tauAct 1]);
delay = 0.2;
% Step response
figure();
step(sysAct);
grid on;
% The settling time is 0.003s

%% Control
% Extended system
Ae = [-rho*Area*Cv*v0/m 0; -C 0];
Be = [B; 0]; % we do not include the noise theta
Ce = [C 0];
sysExt = ss(Ae, Be, Ce, D);
% Controllability
controllabilityExt = rank(ctrb(sysExt)); 
% CONTROLLABLE as rank = 2 

% Pole placement
% The closed-loop extended system is of order 2. We want it to have a settling time
% of 10s. We do not need an observer as the only state is measurable.
st = 6;
pole1 = -3/st;
pole2 = 10*pole1; % for dynamics of system
Pe = [pole1 pole2];
% Find the gain
Fe = acker(Ae, Be, Pe);
F = Fe(1); % gain for state
H = Fe(2); % gain for integral

%% Simulation data
v0Time = [1 3 5 10 15 20 25 30 35 40]';
vRefTime = zeros(10, 2);
for i = 1:10
    vRefTime(i, 1) = 15*(i - 1); % keep for 15s
    vRefTime(i, 2) = v0Time(i) + 0.5;
end