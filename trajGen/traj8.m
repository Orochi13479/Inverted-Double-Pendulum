% Define parameters of double inverted pendulum system
L1 = 0.195;       % Link 1 length (m)
L2 = 0.215;       % Link 2 length (m)
m1 = 0.36;        % Link 1 mass (kg)
m2 = 0.21;        % Link 2 mass (kg)
g = 9.8;          % gravity (m/s^2)

% Known positions 
data1 = readmatrix('inverted_damped_cosine_wave.csv');

% Time vector for simulation
t_sim = data1(:, 1);  % First column contains time values
q1_desired = data1(:, 2);  % Second column contains inverted y values
q2_desired = data1(:, 3);

% Pre-compute desired velocities and accelerations
dt = mean(diff(t_sim));
q1_dot_desired = [0; diff(q1_desired) / dt];  % Numerical differentiation
q2_dot_desired = [0; diff(q2_desired) / dt];
q1_dot_dot_desired = [0; diff(q1_dot_desired) / dt];
q2_dot_dot_desired = [0; diff(q2_dot_desired) / dt];

% Initial conditions for simulation
q1_dot_0 = q1_dot_desired(1);
q2_dot_0 = q2_dot_desired(1);
y0 = [q1_dot_0; q2_dot_0; q1_dot_dot_desired(1); q2_dot_dot_desired(1)];

% Time span
tspan = t_sim;

% Set ODE solver options to handle stiff equations
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

% Solve the ODE using a stiff solver
[t, y] = ode15s(@(t, y) double_pendulum_ode(t, y, q1_desired, q2_desired, q1_dot_desired, q2_dot_desired, t_sim, m1, m2, L1, L2, g), tspan, y0, options);

% Extract results
q1_sim = y(:, 1);
q2_sim = y(:, 3);
q1_dot_sim = y(:, 2);
q2_dot_sim = y(:, 4);

% Ensure consistent dimensions
len = length(t_sim);
q1_sim = q1_sim(1:len);
q2_sim = q2_sim(1:len);
q1_dot_sim = q1_dot_sim(1:len);
q2_dot_sim = q2_dot_sim(1:len);

% Initialize arrays to store results
q1_dot_dot_sim = zeros(size(t_sim));
q2_dot_dot_sim = zeros(size(t_sim));
tau1 = zeros(size(t_sim));
tau2 = zeros(size(t_sim));

for i = 1:length(t)
    % Interpolate positions
    q1 = interp1(t_sim, q1_desired, t(i), 'linear');
    q2 = interp1(t_sim, q2_desired, t(i), 'linear');
    
    % Velocities and accelerations
    q1_dot = y(i, 1);
    q2_dot = y(i, 2);
    
    % Mass matrix
    M11 = m1 * L1^2 + m2 * (L1^2 + 2 * L1 * L2 * cos(q2) + L2^2);
    M12 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M21 = M12;
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
    
    % Accelerations
    q_dot_dot = M \ ([-c(1); -c(2)] - g_q);
    
    % Compute torques using inverse dynamics
    tau = M * q_dot_dot + c + g_q;
    
    % Enforce torque limits
    tau = min(max(tau, -0.9), 0.9);  % torque limits
    
    % Store results
    tau1(i) = tau(1);
    tau2(i) = tau(2);
    q1_dot_dot_sim(i) = q_dot_dot(1);
    q2_dot_dot_sim(i) = q_dot_dot(2);
end

% Ensure consistent dimensions for all results
tau1 = tau1(1:len);
tau2 = tau2(1:len);
q1_dot_dot_sim = q1_dot_dot_sim(1:len);
q2_dot_dot_sim = q2_dot_dot_sim(1:len);

%% GENERATE CSV FILE OF TRAJECTORY
% Generate CSV file of Trajectory Generation Data
filename = 'trajectory_data_20.csv';
data = [t_sim(:), q1_sim(:), q1_dot_sim(:), q1_dot_dot_sim(:), tau1(:), q2_sim(:), q2_dot_sim(:), q2_dot_dot_sim(:), tau2(:)];
title_line = 'Time,q1,q1_dot,q1_dot_dot,tau1,q2,q2_dot,q2_dot_dot,tau2';
fid = fopen(filename, 'w');
fprintf(fid, '%s\n', title_line);
fclose(fid);
writematrix(data, filename, 'WriteMode', 'append');

% Plot and animate the results
figure;
hold on;
axis equal;
xlim([-0.5 0.5]);
ylim([-0.5 0.5]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Double Inverted Pendulum Animation');
pendulum1 = plot([0, L1 * sin(q1_sim(1))], [0, -L1 * cos(q1_sim(1))], 'r-', 'LineWidth', 2);
pendulum2 = plot([L1 * sin(q1_sim(1)), L1 * sin(q1_sim(1)) + L2 * sin(q1_sim(1) + q2_sim(1))], ...
                 [-L1 * cos(q1_sim(1)), -L1 * cos(q1_sim(1)) - L2 * cos(q1_sim(1) + q2_sim(1))], 'b-', 'LineWidth', 2);
q1_text = text(0.3, 0.4, sprintf('q1: %.2f rad', q1_sim(1)), 'FontSize', 10, 'Color', 'k');
q2_text = text(0.3, 0.35, sprintf('q2: %.2f rad', q2_sim(1)), 'FontSize', 10, 'Color', 'k');
for i = 2:length(t_sim)
    set(pendulum1, 'XData', [0, L1 * sin(q1_sim(i))], 'YData', [0, -L1 * cos(q1_sim(i))]);
    set(pendulum2, 'XData', [L1 * sin(q1_sim(i)), L1 * sin(q1_sim(i)) + L2 * sin(q1_sim(i) + q2_sim(i))], ...
                   'YData', [-L1 * cos(q1_sim(i)), -L1 * cos(q1_sim(i)) - L2 * cos(q1_sim(i) + q2_sim(i))]);
    set(q1_text, 'String', sprintf('q1: %.2f rad', q1_sim(i)));
    set(q2_text, 'String', sprintf('q2: %.2f rad', q2_sim(i)));
    drawnow;
    pause(0.01);
end

hold off;

%% Generate Graphs for Joint 1 and Joint 2 Angular Position, Velocity, Acceleration, and Torque

% Create a new figure
figure;

% Plot Joint 1 Angular Position
subplot(4, 2, 1);
plot(t_sim, q1_sim, 'b', 'LineWidth', 1.5);
title('Joint 1 Angular Position');
xlabel('Time (s)');
ylabel('Position (rad)');
grid on;

% Plot Joint 1 Angular Velocity
subplot(4, 2, 3);
plot(t_sim, q1_dot_sim, 'r', 'LineWidth', 1.5);
title('Joint 1 Angular Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
grid on;

% Plot Joint 1 Angular Acceleration
subplot(4, 2, 5);
plot(t_sim, q1_dot_dot_sim, 'g', 'LineWidth', 1.5);
title('Joint 1 Angular Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
grid on;

% Plot Joint 1 Torque
subplot(4, 2, 7);
plot(t_sim, tau1, 'k', 'LineWidth', 1.5);
title('Joint 1 Torque');
xlabel('Time (s)');
ylabel('Torque (N.m)');
grid on;

% Plot Joint 2 Angular Position
subplot(4, 2, 2);
plot(t_sim, q2_sim, 'b', 'LineWidth', 1.5);
title('Joint 2 Angular Position');
xlabel('Time (s)');
ylabel('Position (rad)');
grid on;

% Plot Joint 2 Angular Velocity
subplot(4, 2, 4);
plot(t_sim, q2_dot_sim, 'r', 'LineWidth', 1.5);
title('Joint 2 Angular Velocity');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
grid on;

% Plot Joint 2 Angular Acceleration
subplot(4, 2, 6);
plot(t_sim, q2_dot_dot_sim, 'g', 'LineWidth', 1.5);
title('Joint 2 Angular Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
grid on;

% Plot Joint 2 Torque
subplot(4, 2, 8);
plot(t_sim, tau2, 'k', 'LineWidth', 1.5);
title('Joint 2 Torque');
xlabel('Time (s)');
ylabel('Torque (N.m)');
grid on;

