% Define parameters of double inverted pendulum system
L1 = 0.195;       % Link 1 length (m)
L2 = 0.215;       % Link 2 length (m)
m1 = 0.36;        % Link 1 mass (kg)
m2 = 0.21;        % Link 2 mass (kg)
g = 9.8;          % gravity (m/s^2)

% Define time span and initial conditions for simulation
tspan = [0 0.55];   % Time span for simulation
q1_0 = 0;         % Initial angle for the first joint
q2_0 = 0;         % Initial angle for the second joint
q1_dot_0 = 0;     % Initial angular velocity for the first joint
q2_dot_0 = 0;     % Initial angular velocity for the second joint

% Define feedforward torques (example: time-varying)
t_sim = linspace(tspan(1), tspan(2), 100);  % Time vector for simulation
tau1 = 0.028*sin(t_sim); % Example: time-varying torque
tau2 = 0.394*cos(t_sim); % Example: time-varying torque

% Initialize variables to store simulation results
q1_sim = zeros(size(t_sim));
q2_sim = zeros(size(t_sim));
q1_dot_sim = zeros(size(t_sim));
q2_dot_sim = zeros(size(t_sim));
q1_dot_dot_sim = zeros(size(t_sim));
q2_dot_dot_sim = zeros(size(t_sim));


% Simulate the system
for i = 1:numel(t_sim)-1
    % Current state
    q = [q1_0; q2_0; q1_dot_0; q2_dot_0];
    t_current = t_sim(i);
    
    % Solve equations of motion numerically
    [~, q_next] = ode45(@(t, q) double_pendulum_eqns(t, q, L1, L2, m1, m2, g, tau1(i), tau2(i)),...
        [t_current t_sim(i+1)], q);
    
    % Update state for the next iteration
    q1_0 = q_next(end, 1);
    q2_0 = q_next(end, 2);
    q1_dot_0 = q_next(end, 3);
    q2_dot_0 = q_next(end, 4);
    
    % Calculate q_dot_dot
    q_dot_dot = double_pendulum_eqns(t_current, q, L1, L2, m1, m2, g, tau1(i), tau2(i));
    q1_dot_dot = q_dot_dot(1);
    q2_dot_dot = q_dot_dot(2);
    
    % Store simulation results
    q1_sim(i+1) = q1_0;
    q2_sim(i+1) = q2_0;
    q1_dot_sim(i+1) = q1_dot_0;
    q2_dot_sim(i+1) = q2_dot_0;
    q1_dot_dot_sim(i+1) = q1_dot_dot;
    q2_dot_dot_sim(i+1) = q2_dot_dot;
end

%% Create CSV file of Trajectory Generation Data

% Define the filename
filename = 'trajectory_data.csv';

% Transpose each variable and concatenate them into a single matrix
data = [t_sim(:), q1_sim(:), q1_dot_sim(:), q1_dot_dot_sim(:), tau1(:), q2_sim(:), q2_dot_sim(:), q2_dot_dot_sim(:), tau2(:)];

% Define custom variable names as a title
title_line = 'Time,q1,q1_dot,q1_dot_dot,tau1,q2,q2_dot,q2_dot_dot,tau2';

% Write the title to the CSV file without adding a newline character
fid = fopen(filename, 'w');
fprintf(fid, '%s', title_line);
fclose(fid);

% Append data to the CSV file
writematrix(data, filename, 'WriteMode', 'append');


%% Plot Data Points

% Create a new figure to contain all subplots
figure;

% Plot q1_sim
subplot(3, 3, 1);
plot(t_sim, q1_sim);
xlabel('Time');
ylabel('q1 (Radians)');
title('q1 vs Time');

% Plot q1_dot_sim
subplot(3, 3, 2);
plot(t_sim, q1_dot_sim);
xlabel('Time');
ylabel('q1_dot (Radians/Second)');
title('q1_dot vs Time');

% Plot q1_dot_dot_sim
subplot(3, 3, 3);
plot(t_sim, q1_dot_dot_sim);
xlabel('Time');
ylabel('q1_dot_dot (Radians/Second^2)');
title('q1_dot_dot vs Time');

% Plot tau1
subplot(3, 3, 4);
plot(t_sim, tau1);
xlabel('Time');
ylabel('Tau1');
title('Tau1 vs Time');

% Plot q2_sim
subplot(3, 3, 5);
plot(t_sim, q2_sim);
xlabel('Time');
ylabel('q2 (Radians)');
title('q2 vs Time');

% Plot q2_dot_sim
subplot(3, 3, 6);
plot(t_sim, q2_dot_sim);
xlabel('Time');
ylabel('q2_dot (Radians/Second)');
title('q2_dot vs Time');

% Plot q2_dot_dot_sim
subplot(3, 3, 7);
plot(t_sim, q2_dot_dot_sim);
xlabel('Time');
ylabel('q2_dot_dot (Radians/Second^2)');
title('q2_dot_dot vs Time');

% Plot tau2
subplot(3, 3, 8);
plot(t_sim, tau2);
xlabel('Time');
ylabel('Tau2');
title('Tau2 vs Time');

% Plot q1 and q2 on the same plot
subplot(3, 3, 9);
plot(t_sim, q1_sim);
hold on;
plot(t_sim, q2_sim);
hold off;
xlabel('Time');
ylabel('Angle (Radians)');
title('q1 and q2 vs Time');
legend('q1', 'q2');