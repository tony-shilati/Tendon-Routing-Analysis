%%
clc, clearvars


%% Finger information

% Link Lengths (m)
L1 = 0.05;
L2 = 0.03;
L3 = 0.02;

% Joint Angles
theta1 = 45*pi/180;
theta2 = 45*pi/180;

% Joint radii (m)
r1 = 0.0075;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.005;

%% Rigid coupling model
% Coupling specific lengths 
cl1 = L2;
cl3 = 0.007;
cl4 = 0.007;

% Input angle (joint 2 angle)
theta_c1 = theta2;
% Initial coupling angles for calculating cl2
theta_c1i = 0 * pi/180;
theta_c3i = 30 * pi/180;

[theta_c2, theta_c3, cl2] = RigidCouplingAngles(cl1, cl3, cl4, theta2, theta_c1i, theta_c3i);

N = RigidCouplingTransmissionRatio(cl1, cl3, theta_c1, theta_c2, theta_c3);

%% Finger Dynamics

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2 + N*S3];

% Desired Wrench - closed finger
W = [0, 0, 2.18, -27.48, -4.84, 0]';

% Desired Wrench - extended finger
% W = [0, 0, 2.3, 0, 23, 0]';

% Tendon Routing Matrix
R = [r1, r1;
     0,  r2] * 1/r_m;

% Tendon Routing Matrix
R_i = inv(R');

% Inverse Motor Radius Matrix
R_m = eye(2) ./r_m; 

% Joint Torques
tau = J'*W

% Tendon Forces
f = R_m * R_i' * tau

% Back calculate joint torques to verify force solution
tau_check = R * inv(R_m) * f 
