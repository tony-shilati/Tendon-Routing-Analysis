function [f_pi, f, null_coeff, optim_error, tau] = NP1_Optimization(L, theta, R_t, W)
%% Finger information

% Link Lengths (m)
L1 = L(1);
L2 = L(2);
L3 = L(3);

% Joint Angles
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);

%% CFinger Dynamics

% Joint Screw Axes
S1 = [0, 0, 1, 0, 0, 0]';
S2 = [0, 0, 1, L1*sin(theta1), -L1*cos(theta1), 0]';
S3 = [0, 0, 1, L1*sin(theta1)+L2*sin(theta1+theta2), -L1*cos(theta1)-L2*cos(theta1+theta2), 0]';

% Jacobian
J = [S1, S2, S3];

% Tendon routing matrix pseudoinverse
R_t_pi = pinv(R_t');

% Null space of transpose tendon routing matrix
R_tN = null(R_t');

% Joint Torques
tau = J'*W;

% Tendon force with pseudoinverse
f_pi = R_t_pi * tau;


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

problem = createOptimProblem('fmincon','objective',...
    ObjFcnEval,'x0',1,'lb', -500, 'ub',500, ...
    'nonlcon', @(s) force_constraints(f_pi, R_tN, s), ...
    'options',opts);

ms = MultiStart('UseParallel', true, 'Display', 'off');
[s,fval] = run(ms,problem, 20);


f = f_pi + (s * R_tN);

null_coeff = s;

optim_error = norm(tau - R_t'*f);

end


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
c4 = f_pi_cons(4) + 2;
c = [c1; c2; c3; c4];  % Column vector
ceq = [];  % Equality constraints (not used)
end




