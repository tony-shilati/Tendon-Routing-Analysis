% Based on Valero-Cuevas et al. “Large Index-Fingertip Forces Are Produced 
% by Subject-Independent Patterns of Muscle Excitation.” Journal of 
% Biomechanics, August 1998
%%%%%
% Finger joint angles in study:
%       Ad/Ab: Neutral
%       MCP (theta 1): 0°
%       PIP (theta 2): 0°
%       DIP (theta 3): 0°
% Finger Force at Tip:
%       Palmar: 23 N (~5lb)
% Finger link lengths (proximal to distal):
%       L1: 50 mm
%       L2: 30 mm
%       L3: 20 mm
%%%%%

%% Finger Parameters

% Finger link lengths (m)
L1 = 0.050;
L2 = 0.030;
L3 = 0.020;

% Joint angles (rad)
theta_list = [0; 0; 0] * pi/180;

% Joint rates (rad/s)
dtheta_list = [0; 0; 0];

% Joint accelerations (rad/s/s)
ddtheta_list = [0; 0; 0];

% Gravity acceleration
g = [0; 0; 0];

% Home position link frames
M01 = eye(4);
M12 = eye(4); M12(1, 4) = L1;
M23 = eye(4); M23(1, 4) = L2;
M34 = eye(4); M34(1, 4) = L3;

% Force on the tip of the finger
F = 23; %(N)
Ftip = [0, 0, 2.3, 0, F, 0]';

% Inertia Matrices
G1 = zeros(6,6);
G2 = zeros(6,6);
G3 = zeros(6,6);

% list of inertias and link frames
Glist = cat(3, G1, G2, G3);
Mlist = cat(3, M01, M12, M23, M34);

% List of srew axis of joints
Slist = [0, 0, 1, 0, 0, 0;
         0, 0, 1, 0, -L1, 0;
         0, 0, 1, 0, -(L1+L2), 0]';


taulist = InverseDynamics(theta_list, dtheta_list, ddtheta_list, g, ...
                         Ftip, Mlist, Glist, Slist);

disp(taulist)



