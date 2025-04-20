% Class_3 = Filtered Smith Predictor (Continuous)
clear; close all; clc;

% specs
% G(s) = 2/(3s + 1); C = k*(s+z)/s;
% Mp = 0.05; ts = 4;

% Plant variables
nump = 2;                           % plant numerator
denp = [3 1];                       % plant denominator
h = 6.5;                            % transport delay
G = tf(nump,denp);                  % plant tf
Gd = tf(nump,denp,'InputDelay',h);  % delayed plant tf

% Smith Predictor variables
% By rootlocus, z = 0.9839
% By |C*G| = 1, k = 5.3360
z = 0.9839;                         % controller zero
k = 5.3360;                         % controller gain
s = tf('s');                        % Laplace operator
C = k*(s+z)/s;                      % PD controller
Gc = C*G;                           % controlled plant without delay tf
% Hry = (C*G)/(1+C*G)

% Filtered Smith Predictor variables
tau = 1.5;
Tf = 2;
Tn = 3;
Tr = 1.5;           % desired time constant

%beta = (1-(1-Tf/Tn)*(1-Tr/Tn)*exp(-h/Tn))*Tn;
F = tf(1, [tau 1]);                 % filter

beta = (1-exp(-h/Tn)*(1-Tf/Tn)^2)*Tn;
F1 = tf( conv([Tr 1],[beta 1]), conv([tau 1],[tau 1]));         % filter

sim = sim("FSPContinuousSimulink.slx");
fsp_time = sim.fsp_time;

% plot FSP
fsp_u = sim.fsp_u;
fsp_c = sim.fsp_c;
fsp_y = sim.fsp_y;

figure(1);
plot(fsp_time, fsp_u, '--k', 'LineWidth', 1.7), hold on;
plot(fsp_time, fsp_c, 'b', 'LineWidth', 1.7);
plot(fsp_time, fsp_y, 'r', 'LineWidth', 1.7);
title('unit step response');
legend('ref','control signal','g(s) with FSP'), hold off;

% plot FSP with disturbance
fsp_u1 = sim.fsp_u1;
fsp_c1 = sim.fsp_c1;
fsp_y1 = sim.fsp_y1;

figure(2);
plot(fsp_time, fsp_u1, '--k', 'LineWidth', 1.7), hold on;
plot(fsp_time, fsp_c1, 'b', 'LineWidth', 1.7);
plot(fsp_time, fsp_y1, 'r', 'LineWidth', 1.7);
title('unit step response');
legend('ref','control signal','g(s) with FSP e disturbance'), hold off;
