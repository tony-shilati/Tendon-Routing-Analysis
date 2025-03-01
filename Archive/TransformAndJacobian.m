clc, %clear

%% Input Variables

% home position - all joint angles = 0 when every link is parallel to the
% table and to the right of the base

%% change these values
% height_from_table = 15;
% y_desired = 23.1 - height_from_table;               %cm
% x_desired = 15;               %cm
% theta_desired = 100;             %degrees
% 
% x_force_desired = 0;             % N
% y_force_desired = -100;            % N
% torque_desired = -200;         % N-cm

% Link Lengths
a1 = 0.05;                        %m
a2 = 0.03;                        %m
a3 = 0.02;                        %m

% % A small tolerance on end effector orientation error
% eomg = 0.01;
% 
% % A small tolerance on the linear position error
% ev = 0.1;


%% Don't Change these values

% Convert theta
% theta_desired = (theta_desired / 180) * pi;

% Joint angles at home position
q1_home = 0;
q2_home = 0;
q3_home = 0;

% Body frame screw axes in the home position [rad/s, cm/s, cm/s]'
S1 = [0, 0, 1, 0, a1 + a2 + a3, 0]';
S2 = [0, 0, 1, 0, a2 + a3, 0]';
S3 = [0, 0, 1, 0, a3, 0]';
Blist = [S1, S2, S3];

% SE2 matrix describing body frame realtive to space frame
M = [1, 0, 0, a1*cos(q1_home) + a2*cos(q2_home) + a3*cos(q3_home);
     0, 1, 0, a1*sin(q1_home) + a2*sin(q2_home) + a3*sin(q3_home);
     0, 0, 1, 0
     0, 0, 0, 1];

% % Desired end effector pose (body frame) relative to the space frame
% T_desired = [cos(theta_desired), -sin(theta_desired), 0, x_desired;
%              sin(theta_desired), cos(theta_desired), 0, y_desired;
%              0,                   0,                   1, 0
%              0,                   0,                   0, 1];

% thetalist0 = [0, 1, 1]';
% thetalist0 = thetalist;


%% Call to the inverse kinematics functions
%[thetalist, success] = IKinBody(Blist, M, T_desired,thetalist0, eomg, ev);

% q1 = thetalist(1);
% q2 = thetalist(2);
% q3 = thetalist(3);

q1 = 45*pi/180;
q2 = 45*pi/180;
q3 = 10*pi/180;

%% Check forward kinematics

% theta_error = abs((theta_desired - (q1 + q2 + q3)) / theta_desired);
% 
% x_out = a1*cos(q1) + a2*cos(q1 + q2) + a3*cos(q1 + q2 + q3);
% x_error = abs((x_desired - x_out) / x_desired);
% y_out = a1*sin(q1) + a2*sin(q1 + q2) + a3*sin(q1 + q2 + q3);
% y_error = abs((y_desired - y_out) / y_desired);

%% Transformation Matrix for Forward Kinematics

% T = [cos(q1+q2+q3), -sin(q1+q2+q3), a2 * cos(q1+q2) + a1*cos(q1) + a3*cos(q1+q2+q3);
%     sin(q1+q2+q3), cos(q1+q2+q3), a2 * sin(q1+q2) + a1*sin(q1) + a3*sin(q1+q2+q3);
%     0, 0, 1];


%% Jacobian Matrix for Forward Velocity Kinematics

J = [-a2*sin(q1+q2) - a1*sin(q1) - a3*sin(q1+q2+q3), -a2*sin(q1+q2)-a3*sin(q1+q2+q3), -a3*sin(q1+q2+q3);
    a2*cos(q1+q2) + a1*cos(q1) + a3*cos(q1+q2+q3), a2*cos(q1+q2) + a3*cos(q1+q2+q3), a3*cos(q1+q2+q3);
    1, 1, 1];


%% Calculation 
tau = J' * [torque_desired;x_force_desired;y_force_desired];

if success == 1
    disp("Sucessfully converged")
    F;

else
    disp("Unsuccessful")
end


% output in N-cm


%% Icluding max possible torque due to mass of motors

% motor_mass = 0.210;                 % kg
% motor_force = motor_mass * 9.81;    % N
% 
% mass_torque_motor1 = motor_force * (a2 + a3);
% mass_torque_motor2 = motor_force * (a3);

fprintf("Success Value: %i\n", success)
total_motor_torques = F + [mass_torque_motor1, mass_torque_motor2, 0]'.*sign(F);
fprintf("Torque 1: %0.3f\nTorque 2: %0.3f\nTorque 3: %0.3f\n", ...
    total_motor_torques(1), total_motor_torques(2), total_motor_torques(3))

%% Plot the configuration of the robot
figure
axis equal
axis([-1, 30, -1, 30])
hold on

% Motor positions
m1 = [0, 0]';
m2 = [a1*cos(q1), a1*sin(q1)]';
m3 = [a1*cos(q1) + a2*cos(q1 + q2), a1*sin(q1) + a2*sin(q1 + q2)]';
end_effector = [x_out, y_out]';

scatter([m1(1), m2(1), m3(1), end_effector(1)], [m1(2), m2(2), m3(2), end_effector(2)], ...
    MarkerFaceColor='k', MarkerEdgeColor= 'k', SizeData=80)

% Link 1
plot([m1(1), m2(1)], [m1(2), m2(2)], 'LineWidth', 1.5, 'Color', 'k')

% Link 2
plot([m2(1), m3(1)], [m2(2), m3(2)], 'LineWidth', 1.5, 'Color', 'k')

% Link 3
plot([m3(1), end_effector(1)], [m3(2), end_effector(2)], 'LineWidth', 1.5, 'Color', 'k')

% Beer bottle dimensions (cm)
overall_height = 23.1 - height_from_table;
base_width = 6.1;
base_height = (2/3) * overall_height;
neck_width = 2.4;

vertices_x_coords = [x_desired - base_width/2, x_desired - base_width/2, ...
    x_desired - base_width/2, x_desired - neck_width/2, x_desired - neck_width/2, ...
    x_desired - neck_width/2, x_desired - neck_width/2, x_desired + neck_width/2, ...
    x_desired + neck_width/2, ...
    x_desired + neck_width/2, x_desired + base_width/2, x_desired + base_width/2, ...
    x_desired + base_width/2];

vertices_y_coords = [0, base_height, base_height, base_height, base_height, ...
    overall_height, overall_height, overall_height, base_height, base_height, ...
    base_height, base_height, 0];

plot(vertices_x_coords, vertices_y_coords, 'LineWidth', 2, 'Color', 'b')




