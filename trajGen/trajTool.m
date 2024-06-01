% Parameters
lambda1 = 0.5;
omega1 = 8*pi/5;         % Adjust number of oscillations
phi1 = pi;          

lambda2 = 0.1;
omega2 = 8*pi/5;         % Adjust number of oscillations
phi2 = pi;
% Time array
t = linspace(0, 10, 1000)';

% Function
y1 = pi * exp(-lambda1 * t) .* cos(omega1 * t + phi1);
y2 = pi/6 * sin(omega2 * t + phi2);

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

