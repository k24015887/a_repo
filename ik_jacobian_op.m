close all; clear; clc;

% Parameter settings
r1 = 1.0;               % Length of the first link of the robot arm
r2 = 1.0;               % Length of the second link of the robot arm
x_d = [2; 2];           % Desired end-effector position [x, y]

t1 = pi/4;              % Initial angle of joint 1
t2 = -pi/4;             % Initial angle of joint 2

k_values = [0, 3, 10];  % Different damping coefficient k values
max_steps = 50;         % Maximum number of iterations
step_limit = 0.1;       % Maximum step size for each iteration
tol = 1e-3;             % Allowed error tolerance

% Graph initialization
figure;
sgtitle('Damped Least Squares');

% Main loop
for k_idx = 1:length(k_values)
    k = k_values(k_idx);  % Current damping coefficient
    subplot(1, 3, k_idx);
    hold on;
    axis equal;
    xlim([-3, 3]);
    ylim([-3, 3]);
    xlabel('X Position');
    ylabel('Y Position');
    title(['k = ', num2str(k)]);
    
    % Current state initialization
    t1_curr = t1;
    t2_curr = t2;
    x_curr = [0; 0];
    
    % Path tracking
    for step = 1:max_steps
        % Compute forward kinematics to get the current end-effector position
        x1 = r1 * [cos(t1_curr); sin(t1_curr)];
        x2 = x1 + r2 * [cos(t1_curr + t2_curr); sin(t1_curr + t2_curr)];
        x_curr = x2;  % Update the current end position
        
        % Plot the current state of the robot arm
        plot([0, x1(1), x2(1)], [0, x1(2), x2(2)], 'b-o');
        plot(x2(1), x2(2), 'ro', 'MarkerFaceColor', 'r');  % Plot the end-effector
        
        % Calculate the error
        error = x_d - x_curr;
        if norm(error) < tol  % If the error is smaller than the tolerance, stop iteration
            break;
        end
        
        % Compute the Jacobian matrix
        J = ik_jacobian(r1, r2, t1_curr, t2_curr);
        
        % Damped Least Squares inverse matrix
        J_dls = J' * inv(J * J' + k^2 * eye(2));
        delta_theta = J_dls * error;  % Calculate the joint angle adjustments
        
        % Limit the step size to avoid excessive adjustments
        if norm(delta_theta) > step_limit
            delta_theta = delta_theta / norm(delta_theta) * step_limit;
        end
        
        % Update joint angles
        t1_curr = t1_curr + delta_theta(1);
        t2_curr = t2_curr + delta_theta(2);
        
        % Calculate manipulability ellipse and plot
        [U, S, ~] = svd(J * J');  % Singular value decomposition
        theta = linspace(0, 2*pi, 100);  % Ellipse angle
        ellipse = U * sqrt(S) * [cos(theta); sin(theta)];  % Ellipse points
        plot(ellipse(1, :) + x_curr(1), ellipse(2, :) + x_curr(2), 'r', 'LineWidth', 0.5, 'Color', [1, 0, 0, 0.3]);
    end
end

% Jacobian matrix function
function J = ik_jacobian(r1, r2, t1, t2)
    % Compute the Jacobian matrix for the 2D robot arm
    J = [-r1*sin(t1) - r2*sin(t1 + t2), -r2*sin(t1 + t2);
          r1*cos(t1) + r2*cos(t1 + t2),  r2*cos(t1 + t2)];
end
