%% ROBOTICS STUDIO 2
% AUTUMN SESSION 2024
% DOUBLE INVERTED PENDULUM SYSTEM 
% This script is a draft trajectory plan for the double
% inverted pendulum system

%% Theory: Equation extracted from Modern Robotics
% 
%     % List the name of variables
%     syms q1 q2 q1_dot q2_dot tau1 tau2 q1_dot_dot q2_dot_dot 
% 
%     % Define values 
%     L1 = 0.195;       % Link 1 length (m) (Motor shaft to motor shaft)
%     L2 = 0.215;       % Link 2 length (m) (Motor shaft to end of link)
%     m1 = 0.36;        % Link 1 mass (kg)
%     m2 = 0.21;        % Link 2 mass (kg)
%     g = 9.8;          % gravity (m/s^2)
% 
% 
%     %% EQUATIONS OF MOTION - From Modern Robotics
% 
%     % Symmetric Positive-Defined Mass Matrix 
%     M11 = m1*(L1^2) + m2*((L1^2)+ 2*L1*L2*cos(q2) + (L2^2));
%     M12 = m2*(L1*L2*cos(q2)+(L2^2));
%     M21 = m2*(L1*L2*cos(q2)+(L2^2));
%     M22 = m2*(L2^2);
%     M_q = [M11 M12; M21 M22];
% 
%     % Coriolis and Centripetal Torques Vector
%     c11 = -m2*L1*L2*sin(q2)*(2*q1_dot*q2_dot + (q2_dot^2));
%     c21 = m2*L1*L2*(q1_dot^2)*sin(q2);
%     c = [c11; c21];
% 
%     % Gravitational Torques Vector
%     g_q11 = (m1 + m2)*L1*g*cos(q1) + m2*g*L2*cos(q1 + q2);
%     g_q21 = m2*g*L2*cos(q1 + q2);
%     g_q = [g_q11; g_q21];
% 
%     % Overall Acceleration
%     q_dot_dot = [q1_dot_dot; q2_dot_dot];
% 
%     % Overall Torque
%     tau = [tau1; tau2];
% 
%     % Overall Equation
%     tau = M_q*q_dot_dot*c'*g_q;
% 
%% Code

% Define parameters of double inverted pendulum system
L1 = 0.195;       % Link 1 length (m)
L2 = 0.215;       % Link 2 length (m)
m1 = 0.36;        % Link 1 mass (kg)
m2 = 0.21;        % Link 2 mass (kg)
g = 9.8;          % gravity (m/s^2)

% Define time span and initial conditions for simulation
tspan = [0 10];   % Time span for simulation
q1_0 = 0;         % Initial angle for the first joint
q2_0 = 0;         % Initial angle for the second joint
q1_dot_0 = 0;     % Initial angular velocity for the first joint
q2_dot_0 = 0;     % Initial angular velocity for the second joint

% Solve equations of motion numerically
[t, q] = ode45(@(t, q) double_pendulum_eqns(t, q, L1, L2, m1, m2, g),...
    tspan, [q1_0; q2_0; q1_dot_0; q2_dot_0]);

% Extract joint angles over time
q1 = q(:, 1);
q2 = q(:, 2);

% Define functions for x and y positions of pendulum masses in cartesian
% co-ordinates
x1 = @(t) L1 * sin(q1(t));
y1 = @(t) -L1 * cos(q1(t));
x2 = @(t) L1 * sin(q1(t)) + L2 * sin(q2(t));
y2 = @(t) -L1 * cos(q1(t)) - L2 * cos(q2(t));

% Create animation
figure;
for i = 1:length(t)
    plot(x1(i), y1(i), 'ro', 'MarkerSize', m1 * 10, 'MarkerFaceColor', 'r');
    hold on;
    plot([0, x1(i)], [0, y1(i)], 'r-');
    plot(x2(i), y2(i), 'go', 'MarkerSize', m2 * 10, 'MarkerFaceColor', 'g');
    plot([x1(i), x2(i)], [y1(i), y2(i)], 'g-');
    hold off;
    axis equal;
    title(['Double Pendulum Animation (t = ', num2str(t(i)), ')']);
    xlabel('X Position');
    ylabel('Y Position');
    xlim([-0.5, 0.5]); % Adjust xlim and ylim as needed
    ylim([-0.5, 0.5]);
    pause(0.01); 
end

function dqdt = double_pendulum_eqns(t, q, L1, L2, m1, m2, g)
    q1 = q(1);
    q2 = q(2);
    q1_dot = q(3);
    q2_dot = q(4);

    M11 = m1 * L1^2 + m2 * (L1^2 + 2 * L1 * L2 * cos(q2) + L2^2);
    M12 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M21 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M22 = m2 * L2^2;
    M = [M11, M12; M21, M22];

    c11 = -m2 * L1 * L2 * sin(q2) * (2 * q1_dot * q2_dot + q2_dot^2);
    c21 = m2 * L1 * L2 * q1_dot^2 * sin(q2);
    c = [c11; c21];

    g_q11 = (m1 + m2) * L1 * g * cos(q1) + m2 * g * L2 * cos(q1 + q2);
    g_q21 = m2 * g * L2 * cos(q1 + q2);
    g_q = [g_q11; g_q21];

    tau1 = 0; % You can specify external torques here if needed
    tau2 = 0;

    q_dot_dot = M \ ([-c(1) + tau1; -c(2) + tau2] - g_q);

    dqdt = [q1_dot; q2_dot; q_dot_dot];
end

%% Things to include:
% calculate inertia based on our system model. The inertia from Modern
% Robotics is based on a simplified model where the link has no mass and
% the mass is on a 'ball' at the end of the link.
% Hand designed Model is literally just a sequence of joint angles and
% velocities. Test on simulation (potentially Daniel's simulation to see
% if the joints 'look' feesable. - this is the pass level
% Next step is to come up with a trajectory that is actually feesable in
% real life. This can be done in multiple ways
% - Through recording a trajectory through hand swinging the robot when
%   motors are off
% - Using Daniel simulator to determine input torques/motion??
% - Using the ode45 solver 
% Feedforward Torque
% PID controller
% Include a plot that tracks angular velocities (q1_dot and q2_dot) and a
% plot that tracks joint angles (q1 and q2) 



