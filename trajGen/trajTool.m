%% SYSTEM POSITION TRAJECTORY GENERATION
% This code generates a csv file 'inverted_damped_cosine_wave' that has
% columns for joint 1 positions, joint 2 positions and timesteps. 
% The remainder of trajectory is calculated through running the traj5.m
% file. 

%% trajectory_data_18
%Parameters
lambda1 = 1.2;            % Adjust decay
omega1 = 2*pi/2;         % Adjust number of oscillationsc
phi1 = pi;          

lambda2 = 3;            % Adjust decay
omega2 = 2*pi/2;         % Adjust number of oscillations
phi2 = pi;
% Time array
t = linspace(0, 4,400)';

% Function
y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
y2 = pi/4* sin(omega2 * t + phi2);

%% trajectory_data_17
% %Parameters
% lambda1 = 1.2;            % Adjust decay
% omega1 = 6*pi/2;         % Adjust number of oscillationsc
% phi1 = pi;          
% 
% lambda2 = 3;            % Adjust decay
% omega2 = 6*pi/2;         % Adjust number of oscillations
% phi2 = pi;
% % Time array
% t = linspace(0, 4,400)';
% 
% % Function
% y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
% y2 = 0*pi/4* sin(omega2 * t + phi2);

%% trajectory_data_13
% %Parameters
% lambda1 = 1.2;            % Adjust decay
% omega1 = 6*pi/2;         % Adjust number of oscillationsc
% phi1 = pi;          
% 
% lambda2 = 3;            % Adjust decay
% omega2 = 6*pi/2;         % Adjust number of oscillations
% phi2 = pi;
% % Time array
% t = linspace(0, 4,400)';
% 
% % Function
% y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
% y2 = pi/4* sin(omega2 * t + phi2);

%% trajectory_data_11 & 12
% %Parameters
% lambda1 = 3;            % Adjust decay
% omega1 = 3*pi/2;         % Adjust number of oscillationsc
% phi1 = pi;          
% 
% lambda2 = 3;            % Adjust decay
% omega2 = 3*pi/2;         % Adjust number of oscillations
% phi2 = pi;
% % Time array
% t = linspace(0, 2,200)';
% 
% % Function
% y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
% y2 = pi/4* sin(omega2 * t + phi2);

%% trajectory_data_09
% % Parameters
% lambda1 = 4;            % Adjust decay
% omega1 = pi/2;         % Adjust number of oscillationsc
% phi1 = pi;          
% 
% lambda2 = 0;            % Adjust decay
% omega2 = pi/2;         % Adjust number of oscillations
% phi2 = pi;
% % Time array
% t = linspace(0, 1, 500)';
% 
% % Function
% y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
% y2 = 0*pi/8* sin(omega2 * t + phi2);

% trajectory_data_10
% %Parameters
% lambda1 = 0.5;            % Adjust decay
% omega1 = 7*pi/5;         % Adjust number of oscillationsc
% phi1 = pi;          
% 
% lambda2 = 3;            % Adjust decay
% omega2 = 7*pi/5;         % Adjust number of oscillations
% phi2 = pi;
% % Time array
% t = linspace(0, 5,500)';
% 
% % Function
% y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
% y2 = pi/6* sin(omega2 * t + phi2);

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

% Plot
subplot(1,2,2)
plot(t, y2)
xlabel('Time')
ylabel('Amplitude Joint 2')
title('Sine Wave Joint 2')
grid on;

