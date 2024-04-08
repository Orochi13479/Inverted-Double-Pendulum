% ROBOTICS STUDIO 2
% AUTUMN SESSION 2024
% DOUBLE INVERTED PENDULUM SYSTEM 
% This script is a draft of hand-designed trajectory plan for the double
% inverted pendulum system


% Define system parameters

m1 = 1;      % Mass of the first arm (and motor?) (kg) - still need to get
m2 = 1;      % Mass of the second arm (kg) - still need to get
l1 = 1;      % Length of the first arm (m) - still need to get 
l2 = 1;      % Length of the second arm (m) - still need to get
g = 9.81;    % Gravity (m/s^2)

% Define the system dynamics (non-linear)
% Where x is the state vector of the system and t represents time.
% The function 'double_pendulum_dynamics' calculates the dynamics
% (equations of motion) of the system given its state vector x and the
% parameters m1, m2, l1, l2 and g. 
system = @(t,x) double_pendulum_dynamics(x, m1, m2, l1, l2, g); 

% Define the initial conditions
% Where q1 and q2 are the joint angles and q1_dot and q2_dot are the joint
% velocities
x0 = [0; 0; 0; 0];          % Initial state [q1, q1_dot, q2, q2_dot]

% Define the desired final state
xf = [pi/2; 0; pi/2; 0];    % Final state [q1, q1_dot, q2, q2_dot]

% Define simulation time span 
time_span = [0, 10]; 

% Trajectory planning using linear interpolation
t = linspace(tspan(1), tspan(2), 100);
q1_desired = linspace(x0(1), xf(1), length(t));
q2_desired = linspace(x0(3), xf(3), length(t));

% Control gains (PID)
kp = 100; 
kd = 20;
ki = 0;

% Simulate System
[t_sim, x_sim] = ode45(@(t, x) double_pendulum_control(t, x, system, q1_desired, q2_desired, kp, kd, ki), tspan, x0);

% Plot results
figure;
plot(t_sim, x_sim(:,1), 'b', t_sim, x_sim(:,3), 'r');
xlabel('Time');
ylabel('Angle(radians)');
legend('q1', 'q2');
title('Double Inverted Pendulum Trajectory Path');

% Define system dynamics function
function dxdt = double_pendulum_dynamics(x, m1, m2, l1, l2, g)
    % Extract state value
    q1 = x(1);
    q1_dot = x(2);
    q2 = x(3); 
    q2_dot = x(4);

    % Equations of motion
    dxdt = zeros(4,1);
    dxdt(1) = q1_dot;
    dxdt(2) = (-m2*12*q2_dot^2*sin(q1-q2)-(m1+m2)*g*sin(q1)+m2*12*q2_dot^2*cos(q1-q2)*sin(q1-q2))/(l1*(m1+m2*sin(q1-q2)^2));
    dxdt(3) = q2_dot;
    dxdt(4) = (m2*12*q2_dot^2*sin(q1-q2)+(m1+m2)*(g*sin(q1)*cos(q1-q2) - l1*q1_dot^2*sin(q1-q2))/12/(m2*12*(m1+m2*sin(q1-q2)^2)));

end

% Define control function
function u = double_pendulum_control(t, x, system, q1_desired, q2_desired, kp, kd, ki)
    % Extract state variables
    q1 = x(1);
    q1_dot = x(2);
    q2 = x(3);
    q2_dot = x(4);

    % Desired trajectory
    q1_des = interp1(t_desired, q1_desired, t);
    q2_des = interp1(t_desired, q2_desired, t);

    % Desired velocities
    q1_dot_des = interp1(t_desired, q1_desired_dot, t);
    q2_dot_des = interp1(t_desired, q2_desired_dot, t);

    % Compute errors
    error1 = q1_des - q1;
    error1_dot = q1_dot_des - q1_dot;
    error2 = q2_des - q2;
    error2_dot = q2_dot_des - q2_dot;

    % Compute control input (PID)
    u1 = kp*error1 + kd*error1_dot + ki*error1;
    u2 = kp*error2 + kd*error2_dot + ki*error1;
    
    % Conbine control inputs
    u = [u1; u2];

end



