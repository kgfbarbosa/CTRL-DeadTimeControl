% Class_6 = Simplified Filtered Smith Predictor (Discrete)
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
z = tf('z', Ts);

% discretization
Gz = c2d(G,Ts);                     % discretize nominal plant tf
% Cz = c2d(C,Ts);                     % discretize controller
% Frz = c2d(F1,Ts);                   % discretize FSP filter
% Lz = c2d(L,Ts);                     % discretize delay

b1 = 0.1599;
a1 = 0.92;
tau1 = 1;
alfa1 = 0.5;
[kc1,kr1,b11,b12]=SFSPTuning(b1,a1,tau1,alfa1,hd,Ts);
G1=b1/(z-a1);
Fz=(b11*z^2+b12*z)/(z-alfa1)^2; 
V=zpk(Fz);
S=minreal(G*(1-L*V),1e-4);

% 
% % simulation parameters
% Tref = 1;
% initialreferencevalue = 0;
% finalreferencevalue = 1;
% Tdisturbance = 10;
% initialdisturbancevalue = 0;
% finaldisturbancevalue = 0.2;
% Tnoise = 20;
% initialnoisevalue = 0;
% finalnoisevalue = 0.0;
% Tsim = 40;
% 
% sim = sim("SFSPDiscreteSimulink.slx");
% sfsp_time = sim.sfsp_time;
% 
% % plot SFSP
% sfsp_ref = sim.sfsp_ref;
% sfsp_u = sim.sfsp_u;
% sfsp_c = sim.sfsp_c;
% sfsp_y = sim.sfsp_y;
% 
% figure(1);
% plot(sfsp_time, sfsp_u, '--k', 'LineWidth', 1.7), hold on;
% plot(sfsp_time, sfsp_c, 'b', 'LineWidth', 1.7);
% plot(sfsp_time, sfsp_y, 'r', 'LineWidth', 1.7);
% title('unit step response');
% legend('ref','control signal','g(s) with SFSP'), hold off;