%% INVERTED DOUBLE PENDULUM SYSTEM TRAJECTORY GENERATION
% This file uses hand-designed angular positions, velocities, and
% accelerations and inverse dynamics to determine the necessary
% feedforward torques to achieve the desired trajectory.
% LIMITS OF THE SYSTEM
% (1) Angular Velocity Range: 7500 [rpm] = 785.3981633974482 [rad/s]
% (2) Angular Acceleration Range:125.663706 [rad /(s^2)]
% (4) Torque Limit: 1 [N.m] Peak Torque

% USES INVERTED DAMPED COSINE WAVE TO DETERMINE JOINT 1 POSITIONS - from
% file trajTool.m
% USES SINE WAVE TO DETERMINE JOINT 2 POSITION - from file trajTool.m
% Uses 

%% SECTION 1: Physical parameters of the system
% Define parameters of double inverted pendulum system
L1 = 0.195;       % Link 1 length (m)
L2 = 0.215;       % Link 2 length (m)
m1 = 0.36;        % Link 1 mass (kg)
m2 = 0.21; %0.037;         % Link 2 mass (kg)    0.037 - lighter arm
g = 9.8;          % gravity (m/s^2)

%% SECTION 2: Initializing simulation variables
% Read data from CSV file - generated in trajTool
data = readmatrix('motor_data.csv');

% Time vector for simulation
t_sim = data(:, 1);  % First column contains time values

% Initialise arrays to store torques and other simulation results
tau1 = zeros(size(t_sim));  % Initialise torque array of first motor to zero
tau2 = zeros(size(t_sim));  % Initialise torque array of second motor to zero
q1_sim = zeros(size(t_sim));  % Initialise position array of first motor to zero
q2_sim = zeros(size(t_sim));  % Initialise position array of second motor to zero
q1_dot_sim = zeros(size(t_sim));  % Initialise velocity array of first motor to zero
q2_dot_sim = zeros(size(t_sim));  % Initialise velocity array of second motor to zero
q1_dot_dot_sim = zeros(size(t_sim));  % Initialise acceleration array of first motor to zero
q2_dot_dot_sim = zeros(size(t_sim));  % Initialise acceleration array of second motor to zero

%% SECTION 3: Desired Trajectory - Positions, Velocities and Accelerations

% TRAJECTORY 1
q1_desired_revolutions = data(:, 2);  % Second column contains desired position in revolutions
q2_desired_revolutions = data(:, 6);
q1_dot_desired_revolutions = data(:, 3);  % Third column contains desired velocity in revolutions/s
q2_dot_desired_revolutions = data(:, 7);

% Convert positions from revolutions to radians
q1_desired = q1_desired_revolutions * 2 * pi;
q2_desired = q2_desired_revolutions * 2 * pi;

% Convert velocities from revolutions/s to radians/s
q1_dot_desired = q1_dot_desired_revolutions * 2 * pi;
q2_dot_desired = q2_dot_desired_revolutions * 2 * pi;

% Time step
dt = t_sim(2) - t_sim(1);

% Smoothing function (moving average)
smooth_data = @(data, window_size) filter(ones(1, window_size) / window_size, 1, data);

% Define the window size for the moving average filter
window_size = 500;  % Adjust as needed

% Smooth the desired position data
q1_desired_smooth = smooth_data(q1_desired, window_size);
q2_desired_smooth = smooth_data(q2_desired, window_size);

% Compute smoothed velocities using central differences
q1_dot_desired_smooth = zeros(size(q1_desired_smooth));
q2_dot_desired_smooth = zeros(size(q2_desired_smooth));
q1_dot_desired_smooth(2:end-1) = (q1_desired_smooth(3:end) - q1_desired_smooth(1:end-2)) / (2 * dt);
q2_dot_desired_smooth(2:end-1) = (q2_desired_smooth(3:end) - q2_desired_smooth(1:end-2)) / (2 * dt);
q1_dot_desired_smooth(1) = (q1_desired_smooth(2) - q1_desired_smooth(1)) / dt;
q1_dot_desired_smooth(end) = (q1_desired_smooth(end) - q1_desired_smooth(end-1)) / dt;
q2_dot_desired_smooth(1) = (q2_desired_smooth(2) - q2_desired_smooth(1)) / dt;
q2_dot_desired_smooth(end) = (q2_desired_smooth(end) - q2_desired_smooth(end-1)) / dt;

% Compute smoothed accelerations using central differences
q1_dot_dot_desired_smooth = zeros(size(q1_dot_desired_smooth));
q2_dot_dot_desired_smooth = zeros(size(q2_dot_desired_smooth));
q1_dot_dot_desired_smooth(2:end-1) = (q1_dot_desired_smooth(3:end) - q1_dot_desired_smooth(1:end-2)) / (2 * dt);
q2_dot_dot_desired_smooth(2:end-1) = (q2_dot_desired_smooth(3:end) - q2_dot_desired_smooth(1:end-2)) / (2 * dt);
q1_dot_dot_desired_smooth(1) = (q1_dot_desired_smooth(2) - q1_dot_desired_smooth(1)) / dt;
q1_dot_dot_desired_smooth(end) = (q1_dot_desired_smooth(end) - q1_dot_desired_smooth(end-1)) / dt;
q2_dot_dot_desired_smooth(1) = (q2_dot_desired_smooth(2) - q2_dot_desired_smooth(1)) / dt;
q2_dot_dot_desired_smooth(end) = (q2_dot_desired_smooth(end) - q2_dot_desired_smooth(end-1)) / dt;

%% SECTION 4: Calculate torques using Inverse Dynamics

% Calculate required torques to achieve desired trajectories
for i = 1:length(t_sim)  % Ensure the loop runs for the correct length
    % Calculate required torques to achieve desired trajectories
    t = t_sim(i);
    q1 = q1_desired_smooth(i);
    q2 = q2_desired_smooth(i);
    q1_dot = q1_dot_desired_smooth(i);
    q2_dot = q2_dot_desired_smooth(i);
    q1_dot_dot = q1_dot_dot_desired_smooth(i);
    q2_dot_dot = q2_dot_dot_desired_smooth(i);
    
    % Enforce acceleration limits
    q1_dot_dot = min(max(q1_dot_dot, -125), 125);
    q2_dot_dot = min(max(q2_dot_dot, -125), 125);
    
    % Mass matrix
    M11 = m1 * L1^2 + m2 * (L1^2 + 2 * L1 * L2 * cos(q2) + L2^2);
    M12 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M21 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M22 = m2 * L2^2;
    M = [M11, M12; M21, M22];
    
    % Coriolis and centripetal torques
    c11 = -m2 * L1 * L2 * sin(q2) * (2 * q1_dot * q2_dot + q2_dot^2);
    c21 = m2 * L1 * L2 * q1_dot^2 * sin(q2);
    c = [c11; c21];
    
    % Gravitational torques
    g_q11 = (m1 + m2) * L1 * g * cos(q1) + m2 * g * L2 * cos(q1 + q2);
    g_q21 = m2 * g * L2 * cos(q1 + q2);
    g_q = [g_q11; g_q21];
    
    % Compute torques using inverse dynamics
    q_dot_dot = [q1_dot_dot; q2_dot_dot];
    tau = M * q_dot_dot + c + g_q;
    
    % % Enforce torque limits
    tau = min(max(tau, -0.9), 0.9);  % torque limits
    % 
    % Store results
    tau1(i) = tau(1);
    tau2(i) = tau(2);
    q1_sim(i) = q1;
    q2_sim(i) = q2;
    q1_dot_sim(i) = q1_dot;
    q2_dot_sim(i) = q2_dot;
    q1_dot_dot_sim(i) = q1_dot_dot;
    q2_dot_dot_sim(i) = q2_dot_dot;
end

%% GENERATE CSV FILE OF TRAJECTORY
% Create CSV file of Trajectory Generation Data

% Define the filename
filename = 'trajectory_data_17.csv';

% Transpose each variable and concatenate them into a single matrix
data = [t_sim(:), q1_sim(:), q1_dot_sim(:), q1_dot_dot_sim(:), tau1(:), q2_sim(:), q2_dot_sim(:), q2_dot_dot_sim(:), tau2(:)];

% Define custom variable names as a title
title_line = 'Time,q1,q1_dot,q1_dot_dot,tau1,q2,q2_dot,q2_dot_dot,tau2';

% Write the title to the CSV file without adding a newline character
fid = fopen(filename, 'w');
fprintf(fid, '%s\n', title_line);
fclose(fid);

% Append data to the CSV file
writematrix(data, filename, 'WriteMode', 'append');
%% SIMULATE SYSTEM

% Calculate the real-time time step
dt_real_time = mean(diff(t_sim));  % Assuming uniform time steps

% Animation of the double inverted pendulum
figure;
hold on;
axis equal;
xlim([-0.5 0.5]);
ylim([-0.5 0.5]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Double Inverted Pendulum Animation');

% Define the pendulum links
pendulum1 = plot([0, L1 * sin(q1_sim(1))], [0, -L1 * cos(q1_sim(1))], 'r-', 'LineWidth', 2);
pendulum2 = plot([L1 * sin(q1_sim(1)), L1 * sin(q1_sim(1)) + L2 * sin(q1_sim(1) + q2_sim(1))], ...
                 [-L1 * cos(q1_sim(1)), -L1 * cos(q1_sim(1)) - L2 * cos(q1_sim(1) + q2_sim(1))], 'b-', 'LineWidth', 2);

% Initialize text annotations
q1_text = text(0.3, 0.4, sprintf('q1: %.2f rad', q1_sim(1)), 'FontSize', 10, 'Color', 'k');
q2_text = text(0.3, 0.35, sprintf('q2: %.2f rad', q2_sim(1)), 'FontSize', 10, 'Color', 'k');
q1_dot_text = text(0.3, 0.3, sprintf('q1_dot: %.2f rad/s', q1_dot_sim(1)), 'FontSize', 10, 'Color', 'k');
q2_dot_text = text(0.3, 0.25, sprintf('q2_dot: %.2f rad/s', q2_dot_sim(1)), 'FontSize', 10, 'Color', 'k');

% Animate the pendulum
for i = 1:length(t_sim)
    % Update pendulum links
    set(pendulum1, 'XData', [0, L1 * sin(q1_sim(i))], 'YData', [0, -L1 * cos(q1_sim(i))]);
    set(pendulum2, 'XData', [L1 * sin(q1_sim(i)), L1 * sin(q1_sim(i)) + L2 * sin(q1_sim(i) + q2_sim(i))], ...
                   'YData', [-L1 * cos(q1_sim(i)), -L1 * cos(q1_sim(i)) - L2 * cos(q1_sim(i) + q2_sim(i))]);
    
    % Update text annotations
    set(q1_text, 'String', sprintf('q1: %.2f rad', q1_sim(i)));
    set(q2_text, 'String', sprintf('q2: %.2f rad', q2_sim(i)));
    set(q1_dot_text, 'String', sprintf('q1_dot: %.2f rad/s', q1_dot_sim(i)));
    set(q2_dot_text, 'String', sprintf('q2_dot: %.2f rad/s', q2_dot_sim(i)));
    
    % Pause to create real-time effect
    pause(dt_real_time);
end

hold off;

%% Generate Graphs for Joint 1 and Joint 2 Angular Position, Velocity, Acceleration, and Torque

% Create a new figure
figure;

% Plot Joint 1 Angular Position
subplot(4, 2, 1);  % Create a 4x2 grid, and use the 1st cell
plot(t_sim, q1_sim, 'b', 'LineWidth', 1.5);
title('Joint 1 Angular Position');
xlabel('Time (s)');
ylabel('Position (rad)');
grid on;

% Plot Joint 1 Angular Velocity
subplot(4, 2, 3);  % Create a 4x2 grid, and use the 3rd cell
plot(t_sim, q1_dot_sim, 'r', 'LineWidth', 1.5);
title('Joint 1 Angular Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
grid on;

% Plot Joint 1 Angular Acceleration
subplot(4, 2, 5);  % Create a 4x2 grid, and use the 5th cell
plot(t_sim, q1_dot_dot_sim, 'g', 'LineWidth', 1.5);
title('Joint 1 Angular Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
grid on;

% Plot Joint 1 Torque
subplot(4, 2, 7);  % Create a 4x2 grid, and use the 7th cell
plot(t_sim, tau1, 'k', 'LineWidth', 1.5);
title('Joint 1 Torque');
xlabel('Time (s)');
ylabel('Torque (N.m)');
grid on;

% Plot Joint 2 Angular Position
subplot(4, 2, 2);  % Create a 4x2 grid, and use the 2nd cell
plot(t_sim, q2_sim, 'b', 'LineWidth', 1.5);
title('Joint 2 Angular Position');
xlabel('Time (s)');
ylabel('Position (rad)');
grid on;

% Plot Joint 2 Angular Velocity
subplot(4, 2, 4);  % Create a 4x2 grid, and use the 4th cell
plot(t_sim, q2_dot_sim, 'r', 'LineWidth', 1.5);
title('Joint 2 Angular Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
grid on;

% Plot Joint 2 Angular Acceleration
subplot(4, 2, 6);  % Create a 4x2 grid, and use the 6th cell
plot(t_sim, q2_dot_dot_sim, 'g', 'LineWidth', 1.5);
title('Joint 2 Angular Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
grid on;

% Plot Joint 2 Torque
subplot(4, 2, 8);  % Create a 4x2 grid, and use the 8th cell
plot(t_sim, tau2, 'k', 'LineWidth', 1.5);
title('Joint 2 Torque');
xlabel('Time (s)');
ylabel('Torque (N.m)');
grid on;