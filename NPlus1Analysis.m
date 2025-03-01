clear, clc

%% Description
% This script evaluates the tendon forces for all 512 N+1 tendon routing
% configurations. The tendon routings which cause the minimum overall
% tendon tension and minimum tendon tension variance are indentified.

%% Finger Constants
% Link Lengths (m)
L1 = 0.05;
L2 = 0.03;
L3 = 0.02;

%%% Joint Angles - closed finger
% theta1 = 45*pi/180;
% theta2 = 45*pi/180;
% theta3 = 10*pi/180;

%%% Joint Angles - extended finger
theta1 = 0;
theta2 = 0;
theta3 = 0;

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
% W = [0, 0, 2.18, -27.48, -4.84, 0]';

% Desired Wrench - extended finger
W = [0, 0, 2.3, 0, 23, 0]';

%% Create matirx of tendon routing matrix values
% Each row corresponds to the nine entries of the matrix 
% R = [r11, 0,   0  ;
%      r21, r22, 0  ;    ==> [r11, r21, r22, r31, r32, r33, r41, r42, r43]
%      r31, r32, r33;
%      r41, r42, r43]

r = zeros(512, 9);
r(:, 1) = r1;
r(:, 2) = r1;
r(:, 3) = r2;
r(:, 4) = r1;
r(:, 5) = r2;
r(:, 6) = r3;
r(:, 7) = r1;
r(:, 8) = r2;
r(:, 9) = r3;

for i = 1:512
    binstr = dec2bin(i-1, 9);
    for j = 1:9
        r(i,j) = r(i,j) * (1 - str2num(binstr(j))*2);
    end
end

%% Evaluate all 512 cases for the closed finger case

% tendon_forces = zeros(256, 7);
% tendon_force_variance = zeros(256, 1);
% tendon_max_force = zeros(256, 1);
% null_vector = zeros(256, 5);

viable_counter = 0;
j = 0;

for i = 1:512
    if abs(sum(sign([r(i, 1), r(i,2), r(i, 4), r(i, 7)]))) == 4 || ...
            abs(sum(sign([r(i, 3), r(i,5), r(i, 8)]))) == 3 || ...
            abs(sum(sign([r(i, 6), r(i,9), ]))) == 2
        continue
    end
    j = j+1;
    % Create the tendon routing matrix 
    R = [r(i, 1), 0,       0  ;
         r(i, 2), r(i, 3), 0  ; 
         r(i, 4), r(i, 5), r(i, 6);
         r(i, 7), r(i, 8), r(i, 9)];

    [f_pi, f, null_coeff, optim_error, tau] = NP1_Optimization([L1, L2, L3], ...
        [theta1, theta2, theta3], R, W);

    % Calculate the forces
    tendon_forces(j, :) = [i, null_coeff, optim_error, f'];

    % Calculate the variance in the forces
    tendon_force_variance(j) = std(f);

    % Max abs force in each tendon routing
    tendon_max_force(j) = max(abs(f));

    % Collect viable configurations
    if max(abs(f)) < 150 && std(f) < 300 && f(1) < 0 && f(2) < 0
        viable_counter = viable_counter + 1;
        viable_configs(viable_counter, :) = [i, max(abs(f)), std(f)];
    end
    disp(i)
end

%% Plot the force of each of the 

routing_number = 1:length(tendon_max_force);
plot(routing_number, tendon_force_variance);
figure
plot(routing_number, tendon_max_force);

disp(min(tendon_max_force))
disp(viable_counter)

