clc
clear all

%% Parameters of the system used for the simulation
    
ms  = 34;            % Sprung Mass (kg)
mus = 11;             % Unsprung Mass (kg)
ks  = 6921;          % Suspension Stiffness (N/m)
kus = 81000;          % Wheel stiffness (N/m)
bs  = 0;             % Suspension Inherent Damping coefficient (sec/m)
bus = 0;              % Wheel Inhenrent Damping coefficient (sec/m)
a   = 0.078;          % eccentricity
F   = 0.3;            % frequency
Sim_Time= 60;        % Simulation Initiation/ Time Setting



%% System Dynamics for the system
A = [ 0 1 0 -1 ;
    -ks/ms -bs/ms 0 bs/ms;
      0 0 0 1; 
    ks/mus bs/mus -kus/mus -(bs+bus)/mus];

B = [0  0 ; 
     0 1/ms ; 
    -1  0 ;
    bus/mus -1/mus ];

C = [ 1 0 0 0 ; 
    -ks/ms -bs/ms 0 bs/ms ];


D = [0 0;
     0 0;
     0 0;
     0 0;
     0 0;
     0 1/ms];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controllability
rank(ctrb(A,B))

%% LQR Control law
   
R = 1;                           % Control effort
Q = diag([9990000, 999990, 9500, 0.1]);      % [zs - zus, zs_dot, zus - zr, zus_dot]
K = lqr( A, B(:,2), Q, R )         % Compute LQR gain for the first input (actuator force)
%%