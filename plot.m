% Read the CSV file
%data = csvread('output.csv');  % Replace 'output.csv' with your actual filename
data = csvread('output_traj_optim.csv');
% Extract x, y, z coordinates
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);

% Create a 3D scatter plot
figure;
scatter3(x, y, z, 'filled');
title('3D Scatter Plot');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

rotate3d on;
% Optionally, you can add other settings as needed
grid on;
axis equal;  % Ensure equal scaling on all axes
