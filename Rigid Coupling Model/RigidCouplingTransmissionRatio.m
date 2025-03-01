function N = RigidCouplingTransmissionRatio(l1, l3, theta1, theta2, theta3)
    
    N = ((l1/l3)*sin(theta1-theta2) - sin(theta2)*cos(theta3)) / (sin(theta2 - theta3));
end