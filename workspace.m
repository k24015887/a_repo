% Communication between Arduino and MATLAB 
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2024

close all;
clear all;

% Declare global variables
global hPlot hFig c x y 

% Create GUI
hFig = figure;

% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);
c = 0;

% Initialise geometry of 2-arm robotic system
r1 = 100; % Length of first arm
r2 = 100; % Length of second arm

resolution = 50;        
angle1_range = linspace(0, 180, resolution);  
angle2_range = linspace(0, 360, resolution);  

function T = forward_kinematics(r1, r2, theta1, theta2)
    % Convert angles to radians
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);

    % Compute end-effector position
    x = r1 * cos(theta1) + r2 * cos(theta1 + theta2);
    y = r1 * sin(theta1) + r2 * sin(theta1 + theta2);

    % Homogeneous transformation matrix
    T = [1, 0, 0, x;
         0, 1, 0, y;
         0, 0, 1, 0;
         0, 0, 0, 1];
end


% Preallocate memory for coordinates
x = zeros(1, resolution^2);
y = zeros(1, resolution^2);

for t1 = 1:resolution
    for t2 = 1:resolution
        c = c + 1;
        
        % Compute forward kinematics
        T = forward_kinematics(r1, r2, angle1_range(t1), angle2_range(t2));
        
        % Ensure T is 4x4 matrix
        if size(T, 1) < 4 || size(T, 2) < 4
            error('forward_kinematics returned a non-4x4 matrix');
        end
        
        % Extract x and y from T
        x(c) = T(1,4);
        y(c) = T(2,4);
    end
end

% Filter points to fit within the specified range
valid_points = x >= -78 & x <= 78 & y >= 0 & y <= 156;
x = x(valid_points);
y = y(valid_points);

% Real-time plotting
colour = linspace(1,10,length(x));
scatter(hPlot, x, y, 20, colour, 'filled');

% Set axis limits
xlim([-78, 78]);
ylim([0, 156]);

hold on;
