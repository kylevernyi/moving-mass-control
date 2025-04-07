% Define the initial coordinate triad
origin = [0; 0; 0];
x_axis = [1; 0; 0];
y_axis = [0; 1; 0];
z_axis = [0; 0; 1];

% Plot the initial triad
figure;
% hold on;
% grid on;
quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 1);  hold on; % Red for x-axis
quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 1); % Green for y-axis
quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Blue for z-axis

% Define the rotation angle and axis
theta = 90; % Rotation angle in degrees
axis = 'y'; % Rotation axis

% Convert angle to radians
theta_rad = deg2rad(theta);

% Define the rotation matrix about the y-axis
rotation_matrix_y = [cos(theta_rad), 0, sin(theta_rad);
                     0,             1, 0;
                     -sin(theta_rad), 0, cos(theta_rad)];
rotation_matrix_x = [1, 0, 0;
                     0, cos(theta_rad), sin(theta_rad);
                     0, -sin(theta_rad), cos(theta_rad)];
g_IMU = [9.81; 0; 0]; % before rotation
g_B = [0; 0; 9.81]; % after rotation

% Rotate the coordinate vectors
rotated_x_axis = rotation_matrix_y * x_axis;
rotated_y_axis = rotation_matrix_y * y_axis;
rotated_z_axis = rotation_matrix_y * z_axis;

% Plot the rotated triad
quiver3(origin(1), origin(2), origin(3), rotated_x_axis(1), rotated_x_axis(2), rotated_x_axis(3), '--r', 'LineWidth', 2, 'MaxHeadSize', 1); % Dashed red for rotated x-axis
quiver3(origin(1), origin(2), origin(3), rotated_y_axis(1), rotated_y_axis(2), rotated_y_axis(3), '--g', 'LineWidth', 2, 'MaxHeadSize', 1); % Dashed green for rotated y-axis
quiver3(origin(1), origin(2), origin(3), rotated_z_axis(1), rotated_z_axis(2), rotated_z_axis(3), '--b', 'LineWidth', 2, 'MaxHeadSize', 1); % Dashed blue for rotated z-axis

% Set axis labels and title
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Coordinate Triad and Rotated Triad (90 degrees about Y-axis)');

% Set axis limits for better visualization
axis equal;
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);

% Add legend
legend('X', 'Y', 'Z', 'X_{rotated}', 'Y_{rotated}', 'Z_{rotated}');

hold off;