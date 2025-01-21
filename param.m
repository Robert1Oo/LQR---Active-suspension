% M           = 30 
% m           = 13
% k           = 6921
% kt          = 81000
% c           = 0
% ct          = 0
    
    
a   = 0.4           % eccentricity
F   = 3             % frequency
ms  = 34;           % Sprung Mass (kg)
mus = 11;           % Unsprung Mass (kg)
ks  = 6921;         % Suspension Stiffness (N/m)
kus = 81000;        % Wheel stiffness (N/m)
bs  = 0;            % Suspension Inherent Damping coefficient (sec/m)
bus = 0;            % Wheel Inhenrent Damping coefficient (sec/m)


%% System Dynamics for the Active Suspension system.
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
%% Controllability
rank(ctrb(A,B))

%% LQR Control law

Q = diag([5, 4, 1, 1]);

R = 0.1;

K = lqr( A, B(:,2), Q, R ) 

%% Simulation Initiation/ Time Setting
Sim_Time= 10;

sim('Suspension_model')


