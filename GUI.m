% Communication between Arduino and MATLAB 
% @author         Alejandro Granados
% @organisation   King's College London
% @module         Medical Robotics Hardware Development
% @year           2023

close all
clear all

% declare global variables
global s hInputX hInputY hPlot hFig hTimer c y1 y2 r1 r2

% Arm lengths
r1 = 100;
r2 = 100;

%% Set up
% Create serial port object
s = serialport("COM4", 9600);
configureTerminator(s, "CR/LF");
s.UserData = struct("Data", [], "Count", 1);

% Create GUI
hFig = figure;

% Create input fields for x and y positions
hInputX = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25], 'String', 'Enter X');
hInputY = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25], 'String', 'Enter Y');

% Create button for sending commands
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);

% Create plot area
hPlot = axes('Position', [0.2, 0.6, 0.6, 0.3]);

% Set up variables for real-time plotting
c = [];
y1 = [];
y2 = [];
t0 = now;

% Set up timer for continuously receiving data from microcontroller
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
start(hTimer);
hFig.CloseRequestFcn = @closeGUI;

%% Callback function for sending commands
function sendCommand(~, ~)
    global s hInputX hInputY r1 r2

    % Get X and Y from input fields
    x = str2double(get(hInputX, 'String'));
    y = str2double(get(hInputY, 'String'));

    % Validate input
    if isnan(x) || isnan(y)
        disp('Invalid input. Please enter numerical values for X and Y.');
        return;
    end

    % Perform inverse kinematics to calculate joint angles
    [t1, t2] = inverse_kinematics(r1, r2, x, y);

    % Validate inverse kinematics result
    if isempty(t1) || isempty(t2)
        disp('Target position is unreachable.');
        return;
    end

    % Format and send command string to Arduino
    cmdStr = sprintf("C%.2f,%.2f;", t1, t2);
    write(s, cmdStr, "string");
end

%% Callback function for reading data from microcontroller
function readDataTimer(~, ~)
    global s hPlot c y1 y2 r1 r2

    % Read data from Arduino
    dataStr = readline(s);
    if isempty(dataStr) || dataStr == ""
        return;
    end

    % Parse data values from Arduino
    data = sscanf(dataStr, "%c%f,%f");
    c = [c, data(1)];
    y1 = [y1, data(2)]; % t1
    y2 = [y2, data(3)]; % t2

    % Calculate end effector position using forward kinematics
    [x, y] = forward_kinematics(r1, r2, data(2), data(3));

    % Real-time plotting
    plot(hPlot, x, y, 'ro');
    hold on;
end

%% Callback function for closing the GUI
function closeGUI(~, ~)
    global s hFig hTimer

    % Stop timer
    stop(hTimer);
    delete(hTimer);

    % Close serial port
    delete(s);

    % Close GUI
    delete(hFig);
end

%% Forward Kinematics Function
function [x, y] = forward_kinematics(r1, r2, t1, t2)
    t1 = deg2rad(t1);
    t2 = deg2rad(t2);
    x = r1 * cos(t1) + r2 * cos(t1 + t2);
    y = r1 * sin(t1) + r2 * sin(t1 + t2);
end

%% Inverse Kinematics Function
function [t1, t2] = inverse_kinematics(r1, r2, x, y)
    % Calculate t2 using the cosine law
    d = (x^2 + y^2 - r1^2 - r2^2) / (2 * r1 * r2);
    if abs(d) > 1
        t1 = [];
        t2 = [];
        return; % Unreachable
    end
    t2 = atan2(sqrt(1 - d^2), d); % Elbow up solution

    % Calculate t1
    phi = atan2(y, x);
    psi = atan2(r2 * sin(t2), r1 + r2 * cos(t2));
    t1 = phi - psi;

    % Convert to degrees
    t1 = rad2deg(t1);
    t2 = rad2deg(t2);
end
