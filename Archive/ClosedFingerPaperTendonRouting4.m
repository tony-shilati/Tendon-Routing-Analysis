%% Description
% Analysis of maximum tendon forces in a robot finger with N+1 routing for
% tendon routing 4

clear, clc

%% Finger Geometry
% Link Lengths (m)
L1 = 0.05;
L2 = 0.03;
L3 = 0.02;

%%% Joint Angles - closed finger
theta1 = 45*pi/180;
theta2 = 45*pi/180;
theta3 = 10*pi/180;

% %%% Joint Angles - extended finger
% theta1 = 0;
% theta2 = 0;
% theta3 = 0;

% Joint radii (m)
r1 = 0.0075;
r2 = 0.005;
r3 = 0.004;

%% Finger Dynamics

% Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2, S3];

% Desired Wrench - closed finger
W = [0, 0, 2.18, -27.48, -4.84, 0]';

% Desired Wrench - extended finger
% W = [0, 0, 2.3, 0, 23, 0]';

% Tendon Jacobian
J_t = [-r1,  0,   0;
       -r1, -r2,  0;
        r1,  r2, -r3;
        r1,  r2,  r3];

% Tendon Jacobian Pseduoinverse
J_t_pi = pinv(J_t');

% Null spcae of Tendon Jacobian
J_tN = null(J_t')

% Joint Torques
tau = J'*W

% Tendon Forces
f = J_t_pi * tau;
f = f - (f(end) + 2) * J_tN / J_tN(end)

% Back calculate joint torques to verify force solution
tau_check = J_t' * f 