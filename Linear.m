% Initialize parameters
clear; clc; 
close all;

% Robot arm parameters
r1 = 1;  % Length of the first joint
r2 = 1;  % Length of the second joint
theta1 = 45;  % Initial angle of the first joint (degrees)
theta2 = 45;  % Initial angle of the second joint (degrees)

% Target end-effector position
X_start = [-1; 1];  % Start point (x, y)
X_end = [2; -1];    % End point (x, y)

% Interpolation parameters
n_points_list = [3, 10, 30];  % Different numbers of interpolation points

% Figure initialization
figure;
for k = 1:length(n_points_list)
    % Number of interpolation points
    n_points = n_points_list(k);
    
    % Generate linear interpolation path
    t = linspace(0, 1, n_points);  % Parameter t is uniformly distributed from 0 to 1
    trajectory = X_start + (X_end - X_start) * t;  % Linear interpolation formula
    
    % Initialize Jacobian ellipse
    theta1_list = zeros(1, n_points);  % Store theta1 for each point
    theta2_list = zeros(1, n_points);  % Store theta2 for each point
    
    % Initialize joint angles
    theta1_current = theta1;
    theta2_current = theta2;
    
    % Create subplot
    subplot(1, length(n_points_list), k);
    hold on;
    title(['n = ', num2str(n_points)]);
    xlabel('X Position');
    ylabel('Y Position');
    axis equal;
    xlim([-3, 3]);
    ylim([-3, 3]);
    
    % Path generation loop
    for i = 1:n_points
        % Current target position
        X_target = trajectory(:, i);
        
        % Compute the Jacobian matrix
        J = ik_jacobian(r1, r2, theta1_current, theta2_current);
        
        % Compute the Jacobian pseudo-inverse (damped least squares optional)
        J_inv = pinv(J);
        
        % Update joint velocities
        delta_X = X_target - [r1*cosd(theta1_current) + r2*cosd(theta1_current + theta2_current); 
                              r1*sind(theta1_current) + r2*sind(theta1_current + theta2_current)];
        delta_theta = J_inv * delta_X;
        
        % Update joint angles
        theta1_current = theta1_current + rad2deg(delta_theta(1));
        theta2_current = theta2_current + rad2deg(delta_theta(2));
        
        % Store angles
        theta1_list(i) = theta1_current;
        theta2_list(i) = theta2_current;
        
        % Plot robot arm
        joint1 = [0; 0];  % Base
        joint2 = joint1 + [r1*cosd(theta1_current); r1*sind(theta1_current)];
        end_effector = joint2 + [r2*cosd(theta1_current + theta2_current); r2*sind(theta1_current + theta2_current)];
        
        plot([joint1(1), joint2(1)], [joint1(2), joint2(2)], 'k-', 'LineWidth', 0.5); % First joint
        plot([joint2(1), end_effector(1)], [joint2(2), end_effector(2)], 'k-', 'LineWidth', 0.5); % Second joint
        plot(end_effector(1), end_effector(2), 'ro');  % End effector
        
        % Plot Jacobian ellipse (based on Singular Value Decomposition)
        [U, S, V] = svd(J);
        theta = linspace(0, 2*pi, 100);
        ellipse = U * S * [cos(theta); sin(theta)];
        plot(end_effector(1) + ellipse(1, :), end_effector(2) + ellipse(2, :), 'r-', 'LineWidth', 0.5, 'Color', [1, 0, 0, 0.2]);
    end
    
    % Plot interpolation path
    plot(trajectory(1, :), trajectory(2, :), 'b--', 'LineWidth', 1.5);  % Interpolation path
    plot(trajectory(1, :), trajectory(2, :), 'r.', 'MarkerSize', 10);  % Interpolation points
end

% Define Jacobian matrix calculation function
function J = ik_jacobian(r1, r2, t1, t2)
    % Inputs:
    % r1, r2: Link lengths
    % t1, t2: Joint angles (degrees)
    % Output:
    % J: Jacobian matrix
    J = [-r1*sind(t1) - r2*sind(t1 + t2), -r2*sind(t1 + t2);
          r1*cosd(t1) + r2*cosd(t1 + t2),  r2*cosd(t1 + t2)];
end
