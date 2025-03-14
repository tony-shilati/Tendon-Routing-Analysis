%%% Reults for N+1 routing with no coupling, passive tendon coupling, and
%%% rigid coupling in two configurations


%%
clear, clc
%%%%%%%%%%%%
%%% Contracted Cofiguration - No Coupling
%%%%%%%%%%%%

%%%  Finger information
% Link Lengths (m)
L1 = 0.045;
L2 = 0.032;
L3 = 0.026;

%%% Joint Angles
theta1 = 45*pi/180;
theta2 = 45*pi/180;
theta3 = 10*pi/180;

% Joint radii (m)
r1 = 0.010;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.008;

% Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2, S3];

% Stall wrench
x_tip = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
y_tip = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
r = [x_tip, y_tip, 0]';
f = [-27.48, -4.84, 0]';
m_z = [0, 0, 1]*cross(r, f);

F = [0, 0, m_z, f']';

% No load twist
V_s = [0, 0, 0, -0.359, 0, 0]';

% Tendon Routing Matrix
Pi = [r1, r1, r1, -r1;
      0 , r2, r2, -r2;
      0 , 0,  r3, -r3] * 1/r_m;

[tau_m, f, s, torque_error] = NP1_TorqueOptimization(Pi, J, F, r_m)

% Speed
theta_dot_m = Pi' * pinv(J)*V_s
v_t = theta_dot_m * r_m
speed_error = norm(V_s - J*pinv(Pi')*theta_dot_m)



%%
clear, clc
%%%%%%%%%%%%
%%% Contracted Cofiguration - Passive Tendon Coupling
%%%%%%%%%%%%

%%%  Finger information
% Link Lengths (m)
L1 = 0.045;
L2 = 0.032;
L3 = 0.026;

%%% Joint Angles
theta1 = 45*pi/180;
theta2 = 45*pi/180;
theta3 = 10*pi/180;

% Joint radii (m)
r1 = 0.01;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.008;

N = r2/r3;

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2 + N*S3];

% Stall wrench
x_tip = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
y_tip = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
r = [x_tip, y_tip, 0]';
f = [-27.48, -4.84, 0]';
m_z = [0, 0, 1]*cross(r, f);

F = [0, 0, m_z, f']';

% No load twist
V_s = [0, 0, 0, -0.359, 0, 0]';

% Tendon Routing Matrix
Pi = [-r1, r1, r1;
      0 , r2, -r2] * 1/r_m;

[tau_m, f, s, torque_error] = NP1_TorqueOptimization(Pi, J, F, r_m)

% Speed
theta_dot_m = Pi' * pinv(J)*V_s
v_t = theta_dot_m * r_m
speed_error = norm(V_s - J*pinv(Pi')*theta_dot_m)

%%
clear, clc
%%%%%%%%%%%%
%%% Contracted Cofiguration - Rigid Coupling
%%%%%%%%%%%%

%%%  Finger information
% Link Lengths (m)
L1 = 0.045;
L2 = 0.032;
L3 = 0.026;

%%% Joint Angles
theta1 = 45*pi/180;
theta2 = 45*pi/180;
theta3 = 10*pi/180;

% Joint radii (m)
r1 = 0.01;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.008;

%%% Rigid coupling model
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

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2 + N*S3];

% Stall wrench
x_tip = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
y_tip = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
r = [x_tip, y_tip, 0]';
f = [-27.48, -4.84, 0]';
m_z = [0, 0, 1]*cross(r, f);

F = [0, 0, m_z, f']';

% No load twist
V_s = [0, 0, 0, -0.359, 0, 0]';

% Tendon Routing Matrix
Pi = [-r1, r1, r1;
      0 , r2, -r2] * 1/r_m;

[tau_m, f, s, torque_error] = NP1_TorqueOptimization(Pi, J, F, r_m)

% Speed
theta_dot_m = Pi' * pinv(J)*V_s
v_t = theta_dot_m * r_m
speed_error = norm(V_s - J*pinv(Pi')*theta_dot_m)

%% 
clear, clc
%%%%%%%%%%%%
%%% Extended Cofiguration - No Coupling
%%%%%%%%%%%%

%%%  Finger information
% Link Lengths (m)
L1 = 0.045;
L2 = 0.032;
L3 = 0.026;

%%% Joint Angles
theta1 = 0;
theta2 = 0;
theta3 = 0;

% Joint radii (m)
r1 = 0.01;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.008;

% Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2, S3];

% Stall wrench
x_tip = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
y_tip = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
r = [x_tip, y_tip, 0]';
f = [0, 22.2, 0]';
m_z = [0, 0, 1]*cross(r, f);

F = [0, 0, m_z, f']';

% No load twist
V_s = [0, 0, 0, 0, 0.406, 0]';

% Tendon Routing Matrix
Pi = [r1, r1, r1, -r1;
      0 , r2, r2, -r2;
      0 , 0,  r3, -r3] * 1/r_m;

[tau_m, f, s, torque_error] = NP1_TorqueOptimization(Pi, J, F, r_m)

% Speed
theta_dot_m = Pi' * pinv(J)*V_s
v_t = theta_dot_m * r_m
speed_error = norm(V_s - J*pinv(Pi')*theta_dot_m)

%% 
clear, clc
%%%%%%%%%%%%
%%% Extended Cofiguration - Passive Tendon Coupling
%%%%%%%%%%%%

%%%  Finger information
% Link Lengths (m)
L1 = 0.045;
L2 = 0.032;
L3 = 0.026;

%%% Joint Angles
theta1 = 0;
theta2 = 0;
theta3 = 0;

% Joint radii (m)
r1 = 0.01;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.008;

N = r2/r3;

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2 + N*S3];

% Stall wrench
x_tip = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
y_tip = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
r = [x_tip, y_tip, 0]';
f = [0, 22.2, 0]';
m_z = [0, 0, 1]*cross(r, f);

F = [0, 0, m_z, f']';

% No load twist
V_s = [0, 0, 0, 0, 0.406, 0]';

% Tendon Routing Matrix
Pi = [-r1, r1, r1;
      0 , r2, -r2] * 1/r_m;

[tau_m, f, s, torque_error] = NP1_TorqueOptimization(Pi, J, F, r_m)

% Speed
theta_dot_m = Pi' * pinv(J)*V_s
v_t = theta_dot_m * r_m
speed_error = norm(V_s - J*pinv(Pi')*theta_dot_m)

%% 
clear, clc
%%%%%%%%%%%%
%%% Extended Cofiguration - Rigid Coupling
%%%%%%%%%%%%

%%%  Finger information
% Link Lengths (m)
L1 = 0.045;
L2 = 0.032;
L3 = 0.026;

%%% Joint Angles
theta1 = 0;
theta2 = 0;
theta3 = 0;

% Joint radii (m)
r1 = 0.01;
r2 = 0.0075;
r3 = 0.005;

% Motor radii (m)
r_m = 0.008;

%%% Rigid coupling model
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

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2 + N*S3];

% Stall wrench
x_tip = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
y_tip = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
r = [x_tip, y_tip, 0]';
f = [0, 22.2, 0]';
m_z = [0, 0, 1]*cross(r, f);

F = [0, 0, m_z, f']';

% No load twist
V_s = [0, 0, 0, 0, 0.406, 0]';

% Tendon Routing Matrix
Pi = [-r1, r1, r1;
      0 , r2, -r2] * 1/r_m;

[tau_m, f, s, torque_error] = NP1_TorqueOptimization(Pi, J, F, r_m)

% Speed
theta_dot_m = Pi' * pinv(J)*V_s
v_t = theta_dot_m * r_m
speed_error = norm(V_s - J*pinv(Pi')*theta_dot_m)