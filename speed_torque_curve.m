

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

% Maxon Combo 1 - EC 60 flat 100 W
N = 12;
t_stall = 4.3 * N;
t_cont = 0.3 * 12;
omg_nl = 4300 * 2 * pi / 60 / N;

% Fill in continuous operating area
m = -omg_nl/t_stall;
f = @(x) m*x + omg_nl;

scatter(0, omg_nl, 'o', 'filled', "MarkerFaceColor", 	"#0072BD", ...
    "MarkerEdgeColor", 	"#0072BD", 'SizeData', 100), hold on


plot([0, t_stall], [omg_nl, 0], 'LineWidth', 5, "Color", "#0072BD")

%
fill([0, 0, t_cont, t_cont], [0, omg_nl, f(t_cont), 0], "b", "FaceAlpha", 0.3)




% Maxon Combo 2 - EC 60 flat 80 W

N = 23;
t_stall = 1.69 * N;
t_cont = 0.125 * N;
omg_nl = 5600 * 2 * pi / 60 / N;


m = -omg_nl/t_stall;
f = @(x) m*x + omg_nl;

scatter(0, omg_nl, 'o', 'filled', "MarkerFaceColor", 	"#A2142F", ...
    "MarkerEdgeColor", 	"#A2142F", 'SizeData', 100), hold on



plot([0, t_stall], [omg_nl, 0], 'LineWidth', 5, "Color", "#A2142F")


fill([0, 0, t_cont, t_cont], [0, omg_nl, f(t_cont), 0], "r", "FaceAlpha", 0.3)






% Maxon Combo 3 - EC 45 flat 60 W
N = 43;
t_stall = 0.918 * N;
t_cont = 0.075 * N;
omg_nl = 6250 * 2 * pi / 60 / N;


m = -omg_nl/t_stall;
f = @(x) m*x + omg_nl;

scatter(0, omg_nl, 'o', 'filled', "MarkerFaceColor", 	"#77AC30", ...
    "MarkerEdgeColor", 	"#77AC30", 'SizeData', 100), hold on

plot([0, t_stall], [omg_nl, 0], 'LineWidth', 5, "Color", "#77AC30")

fill([0, 0, t_cont, t_cont], [0, omg_nl, f(t_cont), 0], "g", "FaceAlpha", 0.3)




% Finger actuator  desired
scatter(0, 10.5, 'ko', 'filled', 'SizeData', 100), hold on
scatter(2.88, 0, 'ko', 'filled', 'SizeData', 100)
plot([0, 2.88], [10.5, 0], 'k-', 'LineWidth', 5)

% Current actuator
% scatter(0, 118, 'ro', 'filled'), hold on
% scatter(0.540, 0, 'ro', 'filled')
% plot([0, 0.540], [118, 0], 'r-', 'LineWidth', 2.5)


% Plot properties
legend("", "", "100 W - N = 12:1", ...
    "", "", "  80 W - N = 23:1", "", "", "  60 W - N = 43:1", "", "", "Ideal Finger Actuator")

xlabel('Torque (Nm)', 'FontSize', 17, 'FontName', 'Times New Roman')
ylabel('Speed (rad/s)', 'FontSize', 17, 'FontName', 'Times New Roman')
ax = gca;
ax.FontSize = 21; 
axis([0, 5, 0, 55])
box on
grid on
