%% Delta Robot Inverse Kinematics Analysis and Visualization

clear; clc; close all;

%% 1. Define Robot Parameters

% Geometry Parameters (in millimeters)
f = 250;      % Base equilateral triangle side length
e = 80;       % End-effector equilateral triangle side length
rf = 150;     % Length of upper arms (fixed links)
re = 350;     % Length of lower arms (end-effector arms)

% Derived Parameters
sqrt3 = sqrt(3);
pi3 = pi / 3;

% Base joint positions
base = [0, 0, 0;...
        f/2, -sqrt3*f/2, 0;...
        -f/2, -sqrt3*f/2, 0];

% End-effector joint positions (initially at origin)
end_effector = [0, 0, 0;...
                e/2, -sqrt3*e/2, 0;...
                -e/2, -sqrt3*e/2, 0];

%% 2. Define Desired Path

% Circular trajectory parameters
phi = linspace(0, 2*pi, 100);    % Angle parameter for circular path
radius = 80;                     % Radius of the circular path (in mm)
z_height = -200;                 % Constant Z height (in mm)

% Desired end-effector positions
X = [radius*cos(phi)', radius*sin(phi)', z_height*ones(length(phi),1)];

%% 3. Inverse Kinematics Function

function [theta1, theta2, theta3] = inverse_kinematics_delta(f, e, rf, re, x0, y0, z0)
    % Inverse kinematics for Delta robot
    % Given the end-effector position (x0, y0, z0), compute joint angles theta1, theta2, theta3
    
    % Function to compute theta for one arm
    function theta = compute_theta(x, y, z)
        % Constants
        sqrt3 = sqrt(3);
        pi3 = pi / 3;
        
        % Calculate parameters for the quadratic equation
        y1 = -sqrt3 * e / 3;
        z1 = 0;
        
        % Calculate intermediate values
        y = y - y1;
        a = rf;
        b = re;
        
        % Calculation based on geometry
        E = 2*y*z;
        F = 2*x*z;
        G = x^2 + y^2 + z^2 + a^2 - b^2;
        
        % Quadratic equation: F*theta^2 + E*theta + G = 0
        discriminant = E^2 - 4*F*G;
        
        if discriminant < 0
            theta = [];  % No solution
            return;
        end
        
        % Two possible solutions, we take the positive one
        theta = atan2((-E + sqrt(discriminant)), (2*F));
        
        % Convert to degrees
        theta = rad2deg(theta);
    end

    % Compute theta1 for first arm
    theta1 = compute_theta(x0, y0, z0);
    
    % Compute theta2 for second arm (rotated by 120 degrees)
    x2 = X_rot(x0, y0, pi3*2);
    y2 = Y_rot(x0, y0, pi3*2);
    theta2 = compute_theta(x2, y2, z0);
    
    % Compute theta3 for third arm (rotated by -120 degrees)
    x3 = X_rot(x0, y0, -pi3*2);
    y3 = Y_rot(x0, y0, -pi3*2);
    theta3 = compute_theta(x3, y3, z0);
end

% Rotation helper functions
function x_rot = X_rot(x, y, angle)
    x_rot = x*cos(angle) + y*sin(angle);
end

function y_rot = Y_rot(x, y, angle)
    y_rot = -x*sin(angle) + y*cos(angle);
end

%% 4. Compute Inverse Kinematics for Each Point

% Initialize joint angles matrix
th = NaN(length(X), 3);  % Initialize with NaN to handle unreachable positions

% Loop through each desired end-effector position
for i = 1:length(X)
    x = X(i,1);
    y = X(i,2);
    z = X(i,3);
    
    try
        [th1, th2, th3] = inverse_kinematics_delta(f, e, rf, re, x, y, z);
        
        % Check if solution was found
        if isempty(th1) || isempty(th2) || isempty(th3)
            fprintf('No valid solution found at point %d: (%.2f, %.2f, %.2f)\n', i, x, y, z);
        else
            th(i,:) = [th1, th2, th3];
        end
        
    catch ME
        fprintf('Error encountered at point %d: %s\n', i, ME.message);
        th(i,:) = [NaN, NaN, NaN];  % Mark angles as NaN for unreachable positions
    end
end

%% 5. Visualization

figure;
hold on;
grid on;
axis equal;
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('Delta Robot Inverse Kinematics Path');

% Plot the desired end-effector path
plot3(X(:,1), X(:,2), X(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'End-Effector Path');

% Plot the base
plot3(base(:,1), base(:,2), base(:,3), 'k.', 'MarkerSize', 20, 'DisplayName', 'Base Joints');

% Loop through each valid point to plot the robot configuration
for i = 1:length(X)
    if all(~isnan(th(i,:)))
        % Joint angles in degrees
        theta1 = th(i,1);
        theta2 = th(i,2);
        theta3 = th(i,3);
        
        % Compute positions of the robot arms
        [P1, P2, P3] = compute_arm_positions(base, rf, theta1, theta2, theta3, f, e);
        
        % Plot the arms
        plot3([base(1,1), P1(1)], [base(1,2), P1(2)], [base(1,3), P1(3)], 'b-', 'LineWidth', 1);
        plot3([base(2,1), P2(1)], [base(2,2), P2(2)], [base(2,3), P2(3)], 'g-', 'LineWidth', 1);
        plot3([base(3,1), P3(1)], [base(3,2), P3(2)], [base(3,3), P3(3)], 'm-', 'LineWidth', 1);
        
        % Plot the end-effector
        plot3(X(i,1), X(i,2), X(i,3), 'ro', 'MarkerSize', 4);
    end
end

legend;
hold off;

%% Helper Function to Compute Arm Positions

function [P1, P2, P3] = compute_arm_positions(base, rf, theta1, theta2, theta3, f, e)
    % Compute the position of each arm's end based on joint angles
    
    % Convert angles to radians
    th1 = deg2rad(theta1);
    th2 = deg2rad(theta2);
    th3 = deg2rad(theta3);
    
    % Calculate positions for each arm
    P1 = base(1,:) + rf * [cos(th1), sin(th1), -sin(th1)*tan(th1)];
    P2 = base(2,:) + rf * [cos(th2), sin(th2), -sin(th2)*tan(th2)];
    P3 = base(3,:) + rf * [cos(th3), sin(th3), -sin(th3)*tan(th3)];
end