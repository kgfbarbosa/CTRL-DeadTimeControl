% Class_6 = Filtered Smith Predictor (Discrete)
clear; close all; clc;

% specs
% G(s) = 2/(3s + 1); C = k*(s+z)/s;
% Mp = 0.05; ts = 4;

% Plant variables
Ts = 0.25;                          % sample time
s = tf('s');                        % Laplace operator
G = 2/(3*s+1);                      % nominal plant tf
h = 6.5;                            % delay
L = exp(-h*s);
hd = h/Ts;                          % discrete delay

% Smith Predictor variables
% By rootlocus, z = 0.9839
% By |C*G| = 1, k = 5.3360
z = 0.9839;                         % controller zero
k = 5.3360;                         % controller gain
C = k*(s+z)/s;                      % PD controller
Gc = C*G;                           % controlled plant without delay tf
% Fr = 1/((h/100)*s+1);               % FSP filter try


% Filtered Smith Predictor variables
tau = 1.5;
Tf = 2;
Tn = 3;
Tr = 1.5;           % desired time constant

%beta = (1-(1-Tf/Tn)*(1-Tr/Tn)*exp(-h/Tn))*Tn;
F = tf(1, [tau 1]);                 % filter

beta = (1-exp(-h/Tn)*(1-Tf/Tn)^2)*Tn;
F1 = tf( conv([Tr 1],[beta 1]), conv([tau 1],[tau 1]));         % filter

% discretization
Gz = c2d(G,Ts);                     % discretize nominal plant tf
Cz = c2d(C,Ts);                     % discretize controller
Frz = c2d(F1,Ts);                   % discretize FSP filter
Lz = c2d(L,Ts);                     % discretize delay

% simulation parameters
Tref = 1;
initialreferencevalue = 0;
finalreferencevalue = 0.2;
Tdisturbance = 10;
initialdisturbancevalue = 0;
finaldisturbancevalue = 0.2;
Tsim = 40;

sim = sim("FSPDiscreteSimulink.slx");
fsp_time = sim.fsp_time;

% plot SP
fsp_u = sim.fsp_u;
fsp_c = sim.fsp_c;
fsp_y = sim.fsp_y;

figure(1);
plot(fsp_time, fsp_u, '--k', 'LineWidth', 1.7), hold on;
plot(fsp_time, fsp_c, 'b', 'LineWidth', 1.7);
plot(fsp_time, fsp_y, 'r', 'LineWidth', 1.7);
title('unit step response');
legend('ref','control signal','g(s) with SP'), hold off;

% plot FSP
fsp_u1 = sim.fsp_u1;
fsp_c1 = sim.fsp_c1;
fsp_y1 = sim.fsp_y1;

figure(2);
plot(fsp_time, fsp_u1, '--k', 'LineWidth', 1.7), hold on;
plot(fsp_time, fsp_c1, 'b', 'LineWidth', 1.7);
plot(fsp_time, fsp_y1, 'r', 'LineWidth', 1.7);
title('unit step response');
legend('ref','control signal','g(s) with FSP'), hold off;

% plot FSP with disturbance
fsp_u2 = sim.fsp_u2;
fsp_c2 = sim.fsp_c2;
fsp_y2 = sim.fsp_y2;

figure(3);
plot(fsp_time, fsp_u2, '--k', 'LineWidth', 1.7), hold on;
plot(fsp_time, fsp_c2, 'b', 'LineWidth', 1.7);
plot(fsp_time, fsp_y2, 'r', 'LineWidth', 1.7);
title('unit step response');
legend('ref','control signal','g(s) with FSP and disturbance'), hold off;
