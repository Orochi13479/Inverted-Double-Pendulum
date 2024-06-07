%% SYSTEM POSITION TRAJECTORY GENERATION
% This code generates a csv file 'inverted_damped_cosine_wave.csv' that has
% columns for joint 1 positions, joint 2 positions, and timesteps. 
% The remainder of the trajectory is calculated through running the traj5.m
% file. 

%% trajectory_data_18
% Parameters
lambda1 = 1.2;        % Adjust decay
omega1 = 2*pi/4;      % Adjust number of oscillations to 1 oscillation over 4 seconds
phi1 = pi;           

lambda2 = 3;          % Adjust decay
omega2 = 2*pi/4;      % Adjust number of oscillations to 1 oscillation over 4 seconds
phi2 = pi;

% Time array
t = linspace(0, 4, 400)';

% Function
y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
y2 = pi/4 * sin(omega2 * t + phi2);

%% SECTION 2
% Invert y values
y1_inverted = flipud(y1);

% Create a matrix with time and inverted y values
data = [t, y1_inverted, y2];

% Write the matrix to a CSV file
csvwrite('inverted_damped_cosine_wave.csv', data);

% Plot
figure
subplot(1,2,1)
plot(t, y1_inverted)
xlabel('Time')
ylabel('Inverted Amplitude Joint 1')
title('Inverted Damped Cosine Wave Joint 1')
grid on;

subplot(1,2,2)
plot(t, y2)
xlabel('Time')
ylabel('Amplitude Joint 2')
title('Sine Wave Joint 2')
grid on;
