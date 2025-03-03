close, clear, clc

% Finger actuator  desired
scatter(0, 16.8, 'ko', 'filled'), hold on
scatter(1.81, 0, 'ko', 'filled')
plot([0, 1.81], [16.8, 0], 'k-', 'LineWidth', 2.5)

% Current actuaor
scatter(0, 118, 'ro', 'filled'), hold on
scatter(0.540, 0, 'ro', 'filled')
plot([0, 0.540], [118, 0], 'r-', 'LineWidth', 2.5)

% Plot properties
legend("", "","Ideal Finger Actuator", "","", "Testbed Actuation Module")

xlabel('Torque (Nm)', 'FontSize', 17, 'FontName', 'Times New Roman')
ylabel('Speed (rad/s)', 'FontSize', 17, 'FontName', 'Times New Roman')
ax = gca;
ax.FontSize = 21; 
grid on
