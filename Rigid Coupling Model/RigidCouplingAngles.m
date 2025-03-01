function [theta_c2, theta_c3, l2] = RigidCouplingAngles(l1, l3, l4, theta, theta_c1_i, theta_c3_i)

%% Get l2 value
theta2_i = atan((l4 + l3*sin(theta_c3_i) + l1*sin(theta_c1_i)) / (l1*cos(theta_c1_i) + l3*cos(theta_c3_i)));
l2 = (l1*cos(theta_c1_i) + l3*cos(theta_c3_i)) / (cos(theta2_i));


%% Calculate the coupling angles 

% Calculate location of the four bar vertices
    s = (l1^2 + l4^2 - l2^2 - l3^2 + 2*l1*l4*sin(theta)) / (-2*l2*l3);

    alpha = atan2(-sqrt(1-s^2), s);

    K1 = l1*cos(theta) + l2*cos(alpha) - l3;
    K2 = 2*l2*sin(alpha);
    K3 = l1*cos(theta) - (l2*cos(alpha) - l3);
    
    
    theta_c3 = 2*atan((-K2 + sqrt(K2^2 - 4*K1*K3)) / (2*K1));
    theta_c2 = alpha + theta_c3;
end