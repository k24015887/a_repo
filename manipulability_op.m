close all;
clear;
clc;

% Parameters for robot arm
r1 = 1.0; % Length of first link [metre]
r2 = 1.0; % Length of second link
t1 = -0.2; % Initial angle of joint 1
t2 = -1; % Initial angle of joint 2

% Desired position
x_d = [1.5; 1.0]; % Desired x and y coordinates of the end-effector

% Control parameters
damping_factor = 0.01; % Damping factor for DLS
step_limit = 0.1; % Maximum step size for target clamping
gain = 0.3; % Gain for Jacobian transpose method
tolerance = 1e-3; % Position error tolerance
max_iterations = 1000; % Maximum number of iterations
dt = 0.01; % Time step for plotting manipulability
ellipsoid_scale = 0.25; % Scaling factor for ellipsoids

% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);

% Figure setup
figure;
hold on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('X');
ylabel('Y');
title('Robot Arm Motion and Manipulability Ellipsoids');

% Main simulation loop
for i = 1:max_iterations
    % Forward kinematics to calculate current end-effector position
    T = forward_kinematics(r1, r2, t1, t2);
    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y];

    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;

    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) > step_limit
        error = step_limit * (error / norm(error));
    end

    % Compute Jacobian for the current joint angles
    J = ik_jacobian(r1, r2, t1, t2);

    % Calculate manipulability
    manipulability_cal = sqrt(det(J * J'));
    manipulability_values(i) = manipulability_cal;

    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = (J' * J + damping_factor^2 * eye(size(J, 2))) \ J';
    delta_q_dls = inv_J_damped * error;

    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain * J' * error;

    % Select method: uncomment one of the two lines below
    % delta_q = delta_q_dls; % Damped Least Squares method
    delta_q = delta_q_transpose; % Jacobian Transpose method

    % Update joint angles using calculated change
    t1 = t1 + delta_q(1);
    t2 = t2 + delta_q(2);

    % Plot current robot arm position
    plot([0, r1*cos(t1), r1*cos(t1) + r2*cos(t1 + t2)], ...
         [0, r1*sin(t1), r1*sin(t1) + r2*sin(t1 + t2)], 'm-o');

    % Plot the manipulability ellipsoid at the current configuration
    [U, S, ~] = svd(J); % Singular Value Decomposition of the Jacobian
    theta = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * S(1, 1) * cos(theta);
    ellipse_y = ellipsoid_scale * S(2, 2) * sin(theta);
    ellipse = U * [ellipse_x; ellipse_y];

    % Plot the ellipsoid around the current end-effector position
    plot(ellipse(1, :) + x, ellipse(2, :) + y, 'b');

    pause(0.01); % Pause for visualization

    % Check if the end-effector is within the desired tolerance
    if norm(error) < tolerance
        disp('Desired position reached!');
        break;
    end
end

% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');

%% Function Definitions

% Forward kinematics function
function T = forward_kinematics(r1, r2, t1, t2)
    % Compute transformation matrices
    T1 = [cos(t1), -sin(t1), 0, r1*cos(t1);
          sin(t1),  cos(t1), 0, r1*sin(t1);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T2 = [cos(t2), -sin(t2), 0, r2*cos(t2);
          sin(t2),  cos(t2), 0, r2*sin(t2);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T = T1 * T2; % Combined transformation
end

% Jacobian calculation function
function J = ik_jacobian(r1, r2, t1, t2)
    % Compute the Jacobian for a 2-link planar manipulator
    J11 = -r1*sin(t1) - r2*sin(t1 + t2);
    J12 = -r2*sin(t1 + t2);
    J21 =  r1*cos(t1) + r2*cos(t1 + t2);
    J22 =  r2*cos(t1 + t2);
    J = [J11, J12; J21, J22];
end
