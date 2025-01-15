% Link lengths
a1 = 215;
a2 = 213;
a3 = 110;
a4 = 73;

% Joint angle limits (in degrees)
theta1_min = -90; theta1_max = 90;
theta2_min = 15; theta2_max = 45;
theta3_min = -130; theta3_max = -60;
theta4_min = -90; theta4_max = -25;

% Resolution for each joint angle
n = 20;

% Generate grid of joint angles (converted to radians)
theta1 = linspace(deg2rad(theta1_min), deg2rad(theta1_max), n);
theta2 = linspace(deg2rad(theta2_min), deg2rad(theta2_max), n);
theta3 = linspace(deg2rad(theta3_min), deg2rad(theta3_max), n);
theta4 = linspace(deg2rad(theta4_min), deg2rad(theta4_max), n);

% Initialize array to store workspace points (x, y, z, theta_end)
workspace = [];

% Loop over all combinations of joint angles
for i = 1:length(theta1)
    for j = 1:length(theta2)
        for k = 1:length(theta3)
            for l = 1:length(theta4)
                % Calculate forward kinematics for the given joint angles
                T1 = dhTransform(theta1(i), a1, 0, pi/2);
                T2 = dhTransform(theta2(j), 0, a2, 0);
                T3 = dhTransform(theta3(k), 0, a3, 0);
                T4 = dhTransform(theta4(l), 0, a4, 0);

                % End-effector position
                T = T1 * T2 * T3 * T4;
                pos = T(1:3, 4)'; % Extract position
                
                % End-effector orientation (theta2 + theta3 + theta4)
                theta_end = theta2(j) + theta3(k) + theta4(l);
                
                % Convert orientation to degrees
                theta_end_deg = rad2deg(theta_end);
                
                % Store the position and orientation in the workspace array
                workspace = [workspace; pos, theta_end_deg];
            end
        end
    end
end

% Plot the workspace with orientation in degrees as color
figure;
scatter3(workspace(:,1), workspace(:,2), workspace(:,3), 10, workspace(:,4), 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace of 4-DOF Manipulator with Orientation');
colorbar;
caxis([min(workspace(:,4)), max(workspace(:,4))]);
ylabel(colorbar, 'End-Effector Orientation (degrees)');
grid on;
axis equal;

% Function to compute the transformation matrix using DH parameters
function T = dhTransform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end
