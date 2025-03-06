function [tau_m, f, null_coeff, optim_error] = NP1_TorqueOptimization(Pi, J, F, r_m)
%% Finger Forces/Torques

% Joint Torques
tau = J'*F;

% Motor torque particular solution 
tau_mp = pinv(Pi) * tau;


%% Optimization with fmicon

% Start a parallel pool
try
    parpool("local", feature("numcores"))
catch
end

% Call fmincon
ObjFcnEval=@(s)obj_fcn(tau_mp, Pi, s);

rng default % For reproducibility
opts = optimoptions(@fmincon,'Algorithm','interior-point','OptimalityTolerance',1e-5);

problem = createOptimProblem('fmincon','objective',...
    ObjFcnEval,'x0',1,'lb', -500, 'ub',500, ...
    'nonlcon', @(s) force_constraints(tau_mp, Pi, s, r_m), ...
    'options',opts);

ms = MultiStart('UseParallel', true, 'Display', 'off');
[s, fval] = run(ms,problem, 10);

tau_m = tau_mp + null(Pi)*s;
f = tau_m / r_m;

null_coeff = s;

optim_error = norm(tau - Pi * tau_m);

end


%% Define Objective Function and nonlinear constraints
function f=obj_fcn(tau_mp, Pi, s)

F = tau_mp + null(Pi)*s;
f=norm(F);
end



function [c,ceq] = force_constraints(tau_mp, Pi, s, r_m)
lambda = 2;  % Bias force on tendons
f_pi_cons = (tau_mp + null(Pi)*s) / r_m;
for i = 1:length(f_pi_cons)
    c(i, 1) = - f_pi_cons(i) + lambda;
end
ceq = [];  % Equality constraints (not used)
end




