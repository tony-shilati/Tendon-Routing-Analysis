clear, clc, close

%% Fourbar linkage info

l1 = 3;
l3 = 0.7;
l4 = 0.7;

% Calculate l2
theta1 = linspace(0, pi/4, 100);

theta1_i = 0 * pi/180;
theta3_i = 30 * pi/180;

theta2_i = atan((l4 + l3*sin(theta3_i) + l1*sin(theta1_i)) / (l1*cos(theta1_i) + l3*cos(theta3_i)));
% l2 = (l4 + l3 * sin(theta3_i)) / (sin(theta2_i));

theta1_t = 45*pi/180;
[theta2_t, theta3_t] = RigidCouplingAngles(l1, l3, l4, theta1_t, theta1_i, theta3_i);
(theta3_t - theta3_i - pi/2)* 180/pi 

RigidCouplingTransmissionRatio(l1, l3, theta1_t, theta2_t, theta3_t)


%% Four bar calculations

% s = (l1^2 + l4^2 - l2^2 - l3^2 + l1*l4*sin(theta1(i))) / (-2*l2*l3);
% 
% alpha = atan2(-sqrt(1-s^2), s);
% 
% K1 = l2*cos(alpha) - l3;
% K2 = 2*l2*sin(alpha);
% K3 = l1*cos(theta1) + l3 - l2*cos(alpha);
% 
% theta3 = 2*atan((-K2 + sqrt(K2^2 - 4*K1*K3)) / (2*K1));

p1 = [0, 0];
p2 = [0, -l4];

%% Plot the mechanism as it move

N = zeros(1, length(theta1));

plot(0, 0), hold on
axis equal
axis([-1, 4, -1, 4])
ax = gca;
for i = 1:length(theta1)
    % clear axes
    cla(ax)

    % Calculate location of the four bar vertices
    [theta2, theta3, l2] = RigidCouplingAngles(l1, l3, l4, theta1(i), theta1_i, theta3_i); 
    if i == 1
        theta3_ii = theta3;
    end

    p3 = l1 * [cos(theta1(i)), sin(theta1(i))];
    p4 = p2 + l2 * [cos(theta2), sin(theta2)];

    % Plot the four bar linkage
    scatter(p1(1), p1(2), 'ko', 'filled')
    scatter(p2(1), p2(2), 'ko', 'filled')
    scatter(p3(1), p3(2), 'ko', 'filled')
    scatter(p4(1), p4(2), 'ko', 'filled')

    plot([p1(1), p3(1)], [p1(2), p3(2)], 'k-', 'LineWidth', 1.5)
    plot([p1(1), p2(1)], [p1(2), p2(2)], 'k-', 'LineWidth', 1.5)
    plot([p2(1), p4(1)], [p2(2), p4(2)], 'k-', 'LineWidth', 1.5)
    plot([p3(1), p4(1)], [p3(2), p4(2)], 'k-', 'LineWidth', 1.5)

    % Plot link3 of the finger
    link3 = p3 + 1.5*[cos(theta3 - theta3_i), sin(theta3 - theta3_i)];
    plot([p3(1), link3(1)], [p3(2), link3(2)], 'k-', 'LineWidth', 1.5)

    circle(0, 0, 0.75);
    circle(p3(1), p3(2), 0.75);

    % disp("l3: ")
    % disp(l3 - norm(p3-p4))
    % disp("l2: ")
    % disp(l2 - norm(p2-p4))
    % disp("-----")

    N(i) = RigidCouplingTransmissionRatio(l1, l3, theta1(i), theta2, theta3);

    % wait for a quarter of a second
    pause(0.000001)
end

figure
plot(theta1, N)




%% Other functions
function h = circle(x, y, r)

    th = 0:pi/50:2*pi; 

    xunit = r * cos(th) + x;

    yunit = r * sin(th) + y;

    h = plot(xunit, yunit, 'k-', "LineWidth", 1.5); 

end


