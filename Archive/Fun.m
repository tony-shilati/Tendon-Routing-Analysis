l1 = 3;
l3 = linspace(0.3, 0.6, 100);
l4 = 0.5;


theta1_i = 0 * pi/180;
theta3_i = linspace(10, 60, 100) * pi/180;

N = zeros(1, length(l3) * length(theta3_i));

for i = 1:length(l3)
    for j = 1:length(theta3_i)
        theta2_i = atan((l4 + l3(i)*sin(theta3_i(j)) + l1*sin(theta1_i)) / (l1*cos(theta1_i) + l3(i)*cos(theta3_i(j))));
        % l2 = (l4 + l3 * sin(theta3_i)) / (sin(theta2_i));
        
        theta1_t = 45*pi/180;
        try
        [theta2_t, theta3_t] = RigidCouplingAngles(l1, l3(i), l4, theta1_t, theta1_i, theta3_i(j));
        catch 
        end
        (theta3_t - theta3_i(j) - pi/2) * 180/pi ;
        
        try
        N(i) = RigidCouplingTransmissionRatio(l1, l3(i), theta1_t, theta2_t, theta3_t);
        catch 
        end
    end
end

% Plot


