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

% Joint radii (m)
r1 = 0.0075;
r2 = 0.005;
r3 = 0.004;

% Motor radii (m)
r_m = 0.005;

%% Finger Dynamics

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1 + r2/r3, L1*sin(theta1) + (r2/r3)*(L1*sin(theta1)+L2*sin(theta1+theta2)), ...
    -L1*cos(theta1) + (r2/r3)*(-L1*cos(theta1)-L2*cos(theta1+theta2)), 0]';

% Jacobian
J = [S1, S2];

% Desired Wrench
W = [0, 0, 2.18, -27.48, -4.84, 0]';

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


