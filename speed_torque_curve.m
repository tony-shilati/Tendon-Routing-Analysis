

%% Without coupling 
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

%% With Coupling - continuous torque

close, clear, clc

% Finger actuator  desired
scatter(0, 10.5, 'ko', 'filled', 'SizeData', 100), hold on
scatter(2.88, 0, 'ko', 'filled', 'SizeData', 100)
plot([0, 2.88], [10.5, 0], 'k-', 'LineWidth', 5)

% Current actuator
% scatter(0, 118, 'ro', 'filled'), hold on
% scatter(0.540, 0, 'ro', 'filled')
% plot([0, 0.540], [118, 0], 'r-', 'LineWidth', 2.5)

% Maxon Combo 1 - EC 60 flat 100 W
scatter(0, 37.5, 'o', 'filled', "MarkerFaceColor", 	"#0072BD", ...
    "MarkerEdgeColor", 	"#0072BD", 'SizeData', 100), hold on

scatter(3.6, 0, 'o', 'filled', "MarkerFaceColor", 	"#0072BD", ...
    "MarkerEdgeColor", 	"#0072BD", 'SizeData', 100)

plot([0, 3.6], [37.5, 0], '--', 'LineWidth', 5, "Color", "#0072BD")

% Maxon Combo 2 - EC 60 flat 80 W
scatter(0, 27.0, 'o', 'filled', "MarkerEdgeColor", "#A2142F", ...
    "MarkerFaceColor", "#A2142F", 'SizeData', 100), hold on

scatter(2.875, 0, 'o', 'filled', "MarkerEdgeColor", "#A2142F", ...
    "MarkerFaceColor", "#A2142F", 'SizeData', 100)
plot([0, 2.875], [27.0, 0], '--', 'LineWidth', 5, "Color", "#A2142F")

% Maxon Combo 3 - EC 45 flat 60 W
scatter(0, 15.22, 'o', 'filled', "MarkerEdgeColor", "#77AC30", ...
    "MarkerFaceColor",	"#77AC30", 'SizeData', 100), hold on

scatter(3.225, 0, 'o', 'filled', "MarkerEdgeColor", "#77AC30", ...
    "MarkerFaceColor",	"#77AC30", 'SizeData', 100)

plot([0, 3.225], [15.22, 0], '--', 'LineWidth', 5, "Color", "#77AC30")


% Plot properties
legend("", "","Ideal Finger Actuator", "","", "100 W - N = 12:1", ...
    "", "", "80 W - N = 23:1", "", "", "60 W - N = 43:1")

xlabel('Torque (Nm)', 'FontSize', 17, 'FontName', 'Times New Roman')
ylabel('Speed (rad/s)', 'FontSize', 17, 'FontName', 'Times New Roman')
ax = gca;
ax.FontSize = 21; 
grid on
