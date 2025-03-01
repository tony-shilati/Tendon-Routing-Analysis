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
theta3 = 10*pi/180;

% Joint radii (m)
r1 = 0.0075;
r2 = 0.005;
r3 = 0.004;

%% Finger Dynamics

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1 + r2/r3, L1*sin(theta1) + (r2/r3)*(L1*sin(theta1)+L2*sin(theta1+theta2)), ...
    -L1*cos(theta1) + (r2/r3)*(-L1*cos(theta1)-L2*cos(theta1+theta2)), 0]';

% Jacobian
J = [S1, S2];

% Desired Wrench - closed finger
W = [0, 0, 2.18, -27.48, -4.84, 0]';

% Desired Wrench - extended finger
% W = [0, 0, 2.3, 0, 23, 0]';

% Tendon routing matrix
R_t = [r1,  0;
       r1, r2;
       -r1, -r2];

% Tendon routing matrix pseudoinverse
R_t_pi = pinv(R_t');

% Null space of transpose tendon routing matrix
R_tN = null(R_t_pi')

% Joint Torques
tau = J'*W;

% Tendon force with pseudoinverse
f_pi = R_t_pi * tau;


%% Shift Fingers so the geo center is at the origin


% Start a parallel pool
try
    parpool("local", feature("numcores"))
catch
end

tic % Start a timer



%% Optimization with fmicon

% Call fmincon
ObjFcnEval=@(s)obj_fcn(f_pi, R_tN, s);

rng default % For reproducibility
opts = optimoptions(@fmincon,'Algorithm','interior-point','OptimalityTolerance',1e-5);

b = 500;
problem = createOptimProblem('fmincon','objective',...
    ObjFcnEval,'x0',1,'lb', -500, 'ub',500, ...
    'nonlcon', @(s) force_constraints(f_pi, R_tN, s), ...
    'options',opts);

ms = MultiStart('UseParallel', true, 'Display', 'off');
[s,fval] = run(ms,problem, 20);

disp("Tendon force calcualted with pseduoinverse: ")
disp(f_pi)

disp("Tendon force with nullspace optimization: ")
f = f_pi + (s * R_tN); disp(f)

disp("Null space vector coefficient")
disp(s)

disp("To verify that the optimized result is a real answer, " + ...
    "calculate norm(tau - tau_optimized)")
disp(norm(tau - R_t'*f))


%% Define Objective Function and nonlinear constraints
function f=obj_fcn(f_pi, R_tN, s)

F = f_pi + s * R_tN;
f=norm(F);
end

function [c,ceq] = force_constraints(f_pi, R_tN, s)
f_pi_cons = f_pi + s * R_tN;
c1 = f_pi_cons(1) + 2;
c2 = f_pi_cons(2) + 2;
c3 = f_pi_cons(3) + 2;
c = [c1; c2; c3];  % Column vector
ceq = [];  % Equality constraints (not used)
end




