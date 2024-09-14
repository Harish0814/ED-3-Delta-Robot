clc;
clear all; 
close all;

% Circle's parameters
center = [0.04, 0.5];
radius = 0.1;
num_points = 200;  % Number of points along the circle

% Time scaling factor to speed up rotation
rotation_speed_factor = 3;  

% Create scaled angles for faster rotation
angles = linspace(0, 2 * pi * rotation_speed_factor, num_points);

% Calculate x and y coordinates of the circle
x_coords = center(1) + radius * cos(angles);
y_coords = center(2) + radius * sin(angles);

% Linkage lengths
L1 = 0.2;
L2 = 0.4;
L3 = 0.2;
L4 = 0.3;
L5 = 0.3;
L6 = 0.1;

% Initial positions of points A, B, and P
P = struct('x', x_coords(1), 'y', y_coords(1));  % Moving point P
A = struct('x', 0, 'y', 0);  % Fixed point A
B = struct('x', L6, 'y', 0); % Fixed point B

% Initialize figure
figure;
plot(x_coords, y_coords, 'r--'); % Circle path in red dashed line
hold on;

% Plot moving point P
hBP = plot(P.x, P.y, 'yo', 'MarkerFaceColor', 'y');

% Initial calculations of points D, E, and C
D = solveD(A, P, L2, L3 + L5);
E = solveE(D, P, L3);
C = solveC(B, E, L1, L4);

% Plot the initial linkage structure
hL2 = plot([A.x D.x], [A.y D.y], 'g-', 'LineWidth', 2);
hL3 = plot([D.x E.x], [D.y E.y], 'c-', 'LineWidth', 2);
hL5 = plot([E.x P.x], [E.y P.y], 'y-', 'LineWidth', 2);
hL6 = plot([A.x B.x], [A.y B.y], 'b-', 'LineWidth', 2);
hL1 = plot([B.x C.x], [B.y C.y], 'r-', 'LineWidth', 2);
hL4 = plot([C.x E.x], [C.y E.y], 'm-', 'LineWidth', 2);

% Set axis limits and scaling
axis([-1 1 -1 1]);
axis equal;  % Ensure equal scaling of the axes
set(gca, 'YDir', 'reverse'); % Reverse the y-axis for proper orientation

% Animation loop with faster circle rotation
constraint_violations = 0;
for i = 1:num_points
    % Update point P coordinates based on the circle's path
    P.x = x_coords(i);
    P.y = y_coords(i);

    % Update the position of point P in the plot
    set(hBP, 'XData', P.x, 'YData', P.y);

    % Recalculate positions of points D, E, and C
    D = solveD(A, P, L2, L3 + L5);
    E = solveE(D, P, L3);
    C = solveC(B, E, L1, L4);

    % Update linkage positions in the plot
    set(hL2, 'XData', [A.x D.x], 'YData', [A.y D.y]);
    set(hL3, 'XData', [D.x E.x], 'YData', [D.y E.y]);
    set(hL5, 'XData', [E.x P.x], 'YData', [E.y P.y]);
    set(hL1, 'XData', [B.x real(C.x)], 'YData', [B.y real(C.y)]);
    set(hL4, 'XData', [real(C.x) E.x], 'YData', [real(C.y) E.y]);

    % Check constraint for linkage L1 (B to C distance)
    dist_BC = sqrt((B.x - real(C.x))^2 + (B.y - real(C.y))^2);
    if abs(dist_BC - L1) > 1e-6  % Small tolerance for numerical errors
        constraint_violations = constraint_violations + 1;
    end

    % Update the figure to display current frame
    drawnow;
end

% Display the number of constraint violations (if any)
disp(['Number of constraint violations: ', num2str(constraint_violations)]);

% Video recording (optional)
% Uncomment the following lines to create a video
% v = VideoWriter('animation1.mp4', 'MPEG-4');
% open(v);
% for i = 1:num_points
%     frame = getframe(gcf);
%     writeVideo(v, frame);
% end
% close(v);
