% Author: Klysm
% Date: 3/17/25
% Article reproduction

% Article name:
%  Centralized PI controller design method for MIMO
%  processes based on frequency response approximation
% EX1(Industrial Scale Polymerization Reactor (2x2))
%  G1 = [22.89*exp(-0.2*s)/(4.572*s+1), -11.64*exp(-0.4*s)/(1.807*s+1);
%        4.689*exp(-0.2*s)/(2.174*s+1), 5.8*exp(-0.4*s)/(1.801*s+1)];
% Disturbed and Uncertain Example 1

% define script header
clc; clear; close all;
% define Laplace variable 's'
s = tf('s');

% define uncertains in percentage (50 = 50%)
gain_unc = 30;              % gain uncertain value
delay_unc = 30;             % delay uncertain value
sample_unc = 0;            % sample uncertain value

% define uncertain parameters
gU = (1+gain_unc/100);      % gain uncertain (1 + uncertain percentage)
dU = (1+delay_unc/100);     % delay uncertain (1 + uncertain percentage)
sU = (1+sample_unc/100);    % sample uncertain (1 + uncertain percentage)
% define matrix of G1 transfer functions (to simulink)
G1 = [(22.89*gU)*exp(-0.2*dU*s)/(4.572*s+1), ... % G11
    (-11.64*gU)*exp(-0.4*dU*s)/(1.807*s+1);      % G12
      (4.689*gU)*exp(-0.2*dU*s)/(2.174*s+1), ... % G21
      (5.8*gU)*exp(-0.4*dU*s)/(1.801*s+1)];      % G22
% define controllers
C1A = [(0.155+(0.0465/s)), (0.3105+(0.09315/s));    % Ram
       (-0.125-(0.0375/s)), (0.611+(0.1833/s))];
C1B = [(0.207+(0.054/s)), (0.233+(0.062/s));        % Luan
       (-0.160-(0.044/s)), (0.145+(0.122/s))];
C1C = [(0.186+(0.0476/s)), (0.246+(0.0654/s));      % Ghosh
       (-0.144-(0.0385/s)), (0.153+(0.129/s))];

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
G1z = c2d(G1,Ts);                   % discretize plant transfer function
C1zA = c2d(C1A,Ts);                 % discretize 1st controller
C1zB = c2d(C1B,Ts);                 % discretize 2nd controller
C1zC = c2d(C1C,Ts);                 % discretize 3rd controller

% simulation to workspace
sim = sim("example1Simulink.slx");
cpi_time = sim.cpi_time;            % sim time
    % Loop 1
u1 = sim.u1;                        % reference 1
cSignal1A = sim.cSignal1A;          % control signal 1 Ram
cSignal1B = sim.cSignal1B;          % control signal 1 Luan
cSignal1C = sim.cSignal1C;          % control signal 1 Ghosh 
y1A = sim.y1A;                      % output 1 Ram
y1B = sim.y1B;                      % output 1 Luan
y1C = sim.y1C;                      % output 1 Ghosh
    % Loop 2
u2 = sim.u2;                        % reference 2
cSignal2A = sim.cSignal2A;          % control signal 2 Ram
cSignal2B = sim.cSignal2B;          % control signal 2 Luan
cSignal2C = sim.cSignal2C;          % control signal 2 Ghosh
y2A = sim.y2A;                      % output 2 Ram
y2B = sim.y2B;                      % output 2 Luan
y2C = sim.y2C;                      % output 2 Ghosh

% plot 1st example's Centralized PI with load-disturbance and uncertains
    % plot y1
subplot(2, 2, 1);
plot(cpi_time, u1, '--r', 'LineWidth', 1.7), hold on;
plot(cpi_time, y1A, 'b', 'LineWidth', 1.7), hold on;
% axis([0 30, 0 1.6]);
plot(cpi_time, y1B, 'k', 'LineWidth', 4), hold on;
xlabel('Time (hr.)'); ylabel('y1');
plot(cpi_time, y1C, 'm', 'LineWidth', 1.5);
title('Unit step response');
hold off;
    % plot y2
subplot(2, 2, 2);
plot(cpi_time, u2, '--r', 'LineWidth', 1.7), hold on;
plot(cpi_time, y2A, 'b', 'LineWidth', 1.7), hold on;
% axis([0 30, -0.4 1.4]);
plot(cpi_time, y2B, 'k', 'LineWidth', 4), hold on;
xlabel('Time (hr.)'); ylabel('y2');
plot(cpi_time, y2C, 'm', 'LineWidth', 1.5);
title('Unit step response');
legend('ref','Ram','Luan','Ghosh'); % normal plot
hold off;
    % plot u1
subplot(2, 2, 3); % plot y1
plot(cpi_time, cSignal1A, 'b', 'LineWidth', 1.7), hold on;
% axis([0 30, -4 4]);
plot(cpi_time, cSignal1B, 'k', 'LineWidth', 4), hold on;
xlabel('Time (hr.)'); ylabel('u1');
plot(cpi_time, cSignal1C, 'm', 'LineWidth', 1.5);
% title('Unit step response with load-disturbance');
hold off;
    % plot u2
subplot(2, 2, 4); % plot y2
plot(cpi_time, cSignal2A, 'b', 'LineWidth', 1.7), hold on;
% axis([0 30, -0.4 0.7]);
plot(cpi_time, cSignal2B, 'k', 'LineWidth', 4), hold on;
xlabel('Time (hr.)'); ylabel('u2');
plot(cpi_time, cSignal2C, 'm', 'LineWidth', 1.5);
% title('Unit step response with load-disturbance');
hold off;

% INDICES DE DESEMPENHO
% Extrair dados gerados por 'To workspace'
% tout    = ans.tout1;
e1      = sim.e1;
e2      = sim.e2;
e3      = sim.e3;
e4      = sim.e4;
e5      = sim.e5;
e6      = sim.e6;
% dt      = diff(tout);
dt      = diff(cpi_time);
% ISE
ISE1    = (e1(1:end-1).*e1(1:end-1))'*dt;
ISE2    = (e2(1:end-1).*e2(1:end-1))'*dt;
ISE3    = (e3(1:end-1).*e3(1:end-1))'*dt;
ISE4    = (e4(1:end-1).*e4(1:end-1))'*dt;
ISE5    = (e5(1:end-1).*e5(1:end-1))'*dt;
ISE6    = (e6(1:end-1).*e6(1:end-1))'*dt;
% IAE
IAE1    = abs(e1(1:end-1))'*dt;
IAE2    = abs(e2(1:end-1))'*dt;
IAE3    = abs(e3(1:end-1))'*dt;
IAE4    = abs(e4(1:end-1))'*dt;
IAE5    = abs(e5(1:end-1))'*dt;
IAE6    = abs(e6(1:end-1))'*dt;
