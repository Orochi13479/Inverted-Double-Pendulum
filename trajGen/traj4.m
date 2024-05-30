%% INVERTED DOUBLE PENDULUM SYSTEM TRAJECTORY GENERATION
% This file uses hand-designed angular positions, velocities and
% accelerations and inverse dynamics to determine the necessary
% feedforword torques to achieve the desired trajectory.
% LIMITS OF THE SYSTEM
% (1) Angular Velocity Range: 7500 [rpm] = 785.3981633974482 [rad/s]
% (2) Angular Acceleration Range:125.663706 [rad /(s^2)]
% (3) Angular Position Range:
% (4) Torque Limit: 1.7 [N.m] Peak Torque



%% SECTION 1: Physical parameters of the system
% Define parameters of double inverted pendulum system
L1 = 0.195;       % Link 1 length (m)
L2 = 0.215;       % Link 2 length (m)
m1 = 0.36;        % Link 1 mass (kg)
m2 = 0.21;        % Link 2 mass (kg)
g = 9.8;          % gravity (m/s^2)

%% SECTION 2: Initialising simulation variables

% Time vector for simulation
t_sim = linspace(0, 15, 150);                                          % Time vector for simulation

% Initialise arrays to store torques and other simulation results
tau1 = zeros(size(t_sim));                                          % Initialise torque array of first motor to zero
tau2 = zeros(size(t_sim));                                          % Initialise torque array of Second motor to zero
q1_sim = zeros(size(t_sim));                                        % Initialise position array of first motor to zero
q2_sim = zeros(size(t_sim));                                        % Initialise position array of second motor to zero
q1_dot_sim = zeros(size(t_sim));                                    % Initialise velocity array of first motor to zero
q2_dot_sim = zeros(size(t_sim));                                    % Initialise velocity array of second motor to zero
q1_dot_dot_sim = zeros(size(t_sim));                                % Initialise acceleration array of first motor to zero
q2_dot_dot_sim = zeros(size(t_sim));                                % Initialise acceleration array of second motor to zero

%% SECTION 3: Desired Trajectory - Positions, Velocities and Accelerations
% TRAJECTORY 1
% q1_a = linspace(0, -pi/6, 50);
% q1_b = linspace(-pi/6, pi/2, 50);
% q1_c = linspace(pi/2, pi, 50);
% 
% q2_a = linspace(0, -pi/6, 50);
% q2_b = linspace(-pi/6, 0, 50);
% q2_c = linspace(0, 0, 50);
% 
% q1_desired = [q1_a, q1_b, q1_c];                     % Desired position for q1
% q2_desired = [q2_a, q2_b, q2_c];                     % Desired position for q2

% % TRAJECTORY 2
% q1_a = linspace(0, pi, 150);
% q2_a = linspace(0, 0, 150);
% q1_desired = q1_a;                     % Desired position for q1
% q2_desired = q2_a;                     % Desired position for q2

% TRAJECTORY 3
% q1_a = linspace(0, -pi/6, 50);
% q1_b = linspace(-pi/6, pi, 100);
% 
% q2_a = linspace(0, -pi/6, 50);
% q2_b = linspace(-pi/6, 0, 100);
% 
% q1_desired = [q1_a, q1_b];                     % Desired position for q1
% q2_desired = [q2_a, q2_b];                     % Desired position for q2

% TRAJECTORY 4
q1_a = linspace(0, -pi/6, 50);
q1_b = linspace(-pi/6, pi/2, 50);
q1_c = linspace(pi/2, -pi, 50);

q2_a = linspace(0, -pi/6, 50);
q2_b = linspace(-pi/6, pi/6, 50);
q2_c = linspace(pi/6, 0, 50);

q1_desired = [q1_a, q1_b, q1_c];                     % Desired position for q1
q2_desired = [q2_a, q2_b, q2_c];                     % Desired position for q2
% Compute velocities using finite differences
dt = t_sim(2) - t_sim(1);  % Time step
q1_dot_desired = diff(q1_desired) / dt;
q2_dot_desired = diff(q2_desired) / dt;

% Pad the velocities arrays to match the length of the time vector
q1_dot_desired = [q1_dot_desired, 0];  % Add a zero to the end to match the original length
q2_dot_desired = [q2_dot_desired, 0];  % Add a zero to the end to match the original length

% Compute accelerations using finite differences
q1_dot_dot_desired = diff(q1_dot_desired) / dt;
q2_dot_dot_desired = diff(q2_dot_desired) / dt;

% Pad the accelerations arrays to match the length of the time vector
q1_dot_dot_desired = [q1_dot_dot_desired, 0];  % Add a zero to the end to match the original length
q2_dot_dot_desired = [q2_dot_dot_desired, 0];  % Add a zero to the end to match the original length

%% SECTION 4: Calculate torques using Inverse Dynamics

% Calculate required torques to achieve desired trajectories
for i = 1:length(t_sim)  % Ensure the loop runs for the correct length
    % Calculate required torques to achieve desired trajectories
    t = t_sim(i);
    q1 = q1_desired(i);
    q2 = q2_desired(i);
    q1_dot = q1_dot_desired(i);
    q2_dot = q2_dot_desired(i);
    q1_dot_dot = q1_dot_dot_desired(i);
    q2_dot_dot = q2_dot_dot_desired(i);
    
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
filename = 'trajectory_data_1.csv';

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
% % Animation of the double inverted pendulum
% figure;
% hold on;
% axis equal;
% xlim([-0.5 0.5]);
% ylim([-0.5 0.5]);
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% title('Double Inverted Pendulum Animation');
% 
% % Define the pendulum links
% pendulum1 = plot([0, L1 * sin(q1_sim(1))], [0, -L1 * cos(q1_sim(1))], 'r-', 'LineWidth', 2);
% pendulum2 = plot([L1 * sin(q1_sim(1)), L1 * sin(q1_sim(1)) + L2 * sin(q1_sim(1) + q2_sim(1))], ...
%                  [-L1 * cos(q1_sim(1)), -L1 * cos(q1_sim(1)) - L2 * cos(q1_sim(1) + q2_sim(1))], 'b-', 'LineWidth', 2);
% 
% % Animate the pendulum
% for i = 1:length(t_sim)
%     x1 = L1 * sin(q1_sim(i));
%     y1 = -L1 * cos(q1_sim(i));
%     x2 = x1 + L2 * sin(q1_sim(i) + q2_sim(i));
%     y2 = y1 - L2 * cos(q1_sim(i) + q2_sim(i));
% 
%     set(pendulum1, 'XData', [0, x1], 'YData', [0, y1]);
%     set(pendulum2, 'XData', [x1, x2], 'YData', [y1, y2]);
% 
%     drawnow;
% 
%     % Pause to control animation speed
%     pause(0.01);
% end

%% Simulation 2
%% SIMULATE SYSTEM
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
q1_ddot_text = text(0.3, 0.2, sprintf('q1_ddot: %.2f rad/s^2', q1_dot_dot_sim(1)), 'FontSize', 10, 'Color', 'k');
q2_ddot_text = text(0.3, 0.15, sprintf('q2_ddot: %.2f rad/s^2', q2_dot_dot_sim(1)), 'FontSize', 10, 'Color', 'k');
tau1_text = text(0.3, 0.1, sprintf('tau1: %.2f N.m', tau1(1)), 'FontSize', 10, 'Color', 'k');
tau2_text = text(0.3, 0.05, sprintf('tau2: %.2f N.m', tau2(1)), 'FontSize', 10, 'Color', 'k');

% Initialize arrows for velocities and accelerations
velocity_arrow1 = quiver(0, 0, 0, 0, 'r', 'MaxHeadSize', 1, 'LineWidth', 1.5, 'AutoScale', 'off');
velocity_arrow2 = quiver(0, 0, 0, 0, 'r', 'MaxHeadSize', 1, 'LineWidth', 1.5, 'AutoScale', 'off');
acceleration_arrow1 = quiver(0, 0, 0, 0, 'b', 'MaxHeadSize', 1, 'LineWidth', 1.5, 'AutoScale', 'off');
acceleration_arrow2 = quiver(0, 0, 0, 0, 'b', 'MaxHeadSize', 1, 'LineWidth', 1.5, 'AutoScale', 'off');

% Scale factors for arrows (for visualization purposes)
velocity_scale = 0.01;
acceleration_scale = 0.001;

% Animate the pendulum
for i = 1:length(t_sim)
    % Update pendulum positions
    x1 = L1 * sin(q1_sim(i));
    y1 = -L1 * cos(q1_sim(i));
    x2 = x1 + L2 * sin(q1_sim(i) + q2_sim(i));
    y2 = y1 - L2 * cos(q1_sim(i) + q2_sim(i));
    
    set(pendulum1, 'XData', [0, x1], 'YData', [0, y1]);
    set(pendulum2, 'XData', [x1, x2], 'YData', [y1, y2]);
    
    % Update text annotations
    set(q1_text, 'String', sprintf('q1: %.2f rad', q1_sim(i)));
    set(q2_text, 'String', sprintf('q2: %.2f rad', q2_sim(i)));
    set(q1_dot_text, 'String', sprintf('q1_dot: %.2f rad/s', q1_dot_sim(i)));
    set(q2_dot_text, 'String', sprintf('q2_dot: %.2f rad/s', q2_dot_sim(i)));
    set(q1_ddot_text, 'String', sprintf('q1_ddot: %.2f rad/s^2', q1_dot_dot_sim(i)));
    set(q2_ddot_text, 'String', sprintf('q2_ddot: %.2f rad/s^2', q2_dot_dot_sim(i)));
    set(tau1_text, 'String', sprintf('tau1: %.2f N.m', tau1(i)));
    set(tau2_text, 'String', sprintf('tau2: %.2f N.m', tau2(i)));
    
    % Update velocity arrows
    set(velocity_arrow1, 'XData', x1, 'YData', y1, 'UData', velocity_scale * q1_dot_sim(i) * cos(q1_sim(i)), 'VData', velocity_scale * q1_dot_sim(i) * sin(q1_sim(i)));
    set(velocity_arrow2, 'XData', x2, 'YData', y2, 'UData', velocity_scale * q2_dot_sim(i) * cos(q1_sim(i) + q2_sim(i)), 'VData', velocity_scale * q2_dot_sim(i) * sin(q1_sim(i) + q2_sim(i)));
    
    % Update acceleration arrows
    set(acceleration_arrow1, 'XData', x1, 'YData', y1, 'UData', acceleration_scale * q1_dot_dot_sim(i) * cos(q1_sim(i)), 'VData', acceleration_scale * q1_dot_dot_sim(i) * sin(q1_sim(i)));
    set(acceleration_arrow2, 'XData', x2, 'YData', y2, 'UData', acceleration_scale * q2_dot_dot_sim(i) * cos(q1_sim(i) + q2_sim(i)), 'VData', acceleration_scale * q2_dot_dot_sim(i) * sin(q1_sim(i) + q2_sim(i)));
    
    drawnow;
    
    % Pause to control animation speed
    pause(0.01);
end

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
