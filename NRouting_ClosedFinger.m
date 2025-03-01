%% N finger routing for 3 joint finger

clear, clc

%% Finger Geometry
% Link Lengths (m)
L1 = 0.05;
L2 = 0.03;
L3 = 0.02;

%%% Joint Angles
theta1 = 45*pi/180;
theta2 = 45*pi/180;
theta3 = 10*pi/180;

% Joint radii (m)
r1 = 0.0075;
r2 = 0.005;
r3 = 0.004;

% Motor radii (m)
r_m = 0.005;

%% Finger Dynamics

% Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2, S3];

% Desired Wrench
W = [0, 0, 2.18, -27.48, -4.84, 0]';

% Tendon Jacobian
J_t = [r1, 0,   0;
       r1, r2,  0;
       r1, r2,  r3] * 1/r_m;

% Tendon Jacobian inverse
J_t_i = inv(J_t);

% Inverse Motor Radius Matrix
R_inv = eye(3) ./r_m; 

% Joint Torques
tau = J'*W

% Tendon Forces
f = R_inv * J_t_i' * tau

% Back calculate joint torques to verify force solution
tau_check = J_t' * inv(R_inv) * f 


