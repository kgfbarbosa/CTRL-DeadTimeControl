% Author: Klysm
% Date: 3/23/25

% Smith Predictor using Disturbed Example from article:
%  Centralized PI controller design method for MIMO
%  processes based on frequency response approximation
% EX1(Industrial Scale Polymerization Reactor (2x2))
%  G1 = [22.89*exp(-0.2*s)/(4.572*s+1), -11.64*exp(-0.4*s)/(1.807*s+1);
%        4.689*exp(-0.2*s)/(2.174*s+1), 5.8*exp(-0.4*s)/(1.801*s+1)];

% define script header
clc; clear; close all;
% define Laplace variable 's'
s = tf('s');

% define uncertains in percentage (50 = 50%)
gain_unc = 30;               % gain uncertain value
delay_unc = 30;              % delay uncertain value
sample_unc = 0;              % sample uncertain value

% define uncertain parameters
gU = (1+gain_unc/100);      % gain uncertain (1 + uncertain percentage)
dU = (1+delay_unc/100);     % delay uncertain (1 + uncertain percentage)
sU = (1+sample_unc/100);    % sample uncertain (1 + uncertain percentage)
% define matrix of G1 transfer functions (to simulink)
G1 = [(22.89*gU)/(4.572*s+1), ...   % G11 nominal (fast model)
    (-11.64*gU)/(1.807*s+1);        % G12 nominal (fast model)
      (4.689*gU)/(2.174*s+1), ...   % G21 nominal (fast model)
      (5.8*gU)/(1.801*s+1)];        % G22 nominal (fast model)
L = [exp(-0.2*dU*s), 0;             % nominal delay matrix
     0, exp(-0.4*dU*s)];


% define simulation data
Ts = 0.03*sU;                       % sample time
Tsim = 30;                          % simulation time
Tref1 = 0;                          % 1st reference time
Tref2 = 15;                         % 2nd reference time
initialReference1Value = 0;         % initial value of the 1st disturbance
finalReference1Value = 1;           % final value of the 1st disturbance
initialReference2Value = 0;         % initial value of the 2nd disturbance
finalReference2Value = 1;           % final value of the 2nd disturbance
initialDisturbance1Value = 0.1;     % initial value of the 1st disturbance
finalDisturbance1Value = 0.8;       % final value of the 1st disturbance
initialDisturbance2Value = 0.1;     % initial value of the 2nd disturbance
finalDisturbance2Value = 0.8;       % final value of the 2nd disturbance
Gz = c2d(G1,Ts);                    % discretize nominal plant tf matrix
Lz = c2d(L, Ts);                    % discretize nominal delay matrix
Pz = Gz*Lz;
z = tf('z', Ts);

%Controlador 
kc1=1.2;
kc2=1.3;
kc12 = 0.1;
kc21 = 0.25;
kc = [kc1, kc12; kc21, kc2];
kr = [1.05, 0; 0, 1.05];            % reference filter
Frz = [(1.96*z-1.86)/(z-0.904), 0;  % discrete FSP filter
       0, (1.99*z-1.89)/(z-0.904)];
Fc = d2c(Frz);                      % continuous FSP filter estimation
V = zpk(Fc);                        % continuous FSP filter zpk
Sc = minreal(G1*(eye(2)-V*L), 1e-5);% continuous predictor loop
Sz = c2d(Sc, Ts);                   % discretize predictor loop

% simulation to workspace
sim = sim("sfspExample1Simulink.slx");
cpi_time = sim.cpi_time;            % sim time
    % Loop 1
u1 = sim.u1;                        % reference 1
cSignal1A = sim.cSignal1A;          % control signal 1
y1A = sim.y1A;                      % output 1
    % Loop 2
u2 = sim.u2;                        % reference 2
cSignal2A = sim.cSignal2A;          % control signal 2
y2A = sim.y2A;                      % output 2

% plot 1st example's SFS Predictor
    % plot y1
figure(2)
subplot(2, 2, 1);
plot(cpi_time, y1A, 'k', 'LineWidth', 1.7), hold on;
xlabel('Time (hr.)'); ylabel('y1');
title('Unit step response with disturbance'), hold off;
    % plot y2
subplot(2, 2, 2);
plot(cpi_time, y2A, 'k', 'LineWidth', 1.7), hold on;
xlabel('Time (hr.)'); ylabel('y2');
title('Unit step response with disturbance'), hold off;
    % plot u1
subplot(2, 2, 3);
plot(cpi_time, cSignal1A, 'k', 'LineWidth', 1.7), hold on;
xlabel('Time (hr.)'); ylabel('u1'), hold off;
    % plot u2
subplot(2, 2, 4);
plot(cpi_time, cSignal2A, 'k', 'LineWidth', 1.7), hold on;
xlabel('Time (hr.)'); ylabel('u2'), hold off;
