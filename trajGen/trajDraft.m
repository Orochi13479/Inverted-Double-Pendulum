
%% HAND DESIGNED TRAJECTORY 
% Hand design joint position and angular velocity states so that the
% pendulum swings up from downward position to balancing in an upright
% position.
% Angular Velocity [rad/s]
% Formula: (2*pi*RPM)/60
% Torque Limitations result in motor only being able to lift double
% pendulum to an angle of 30 degrees (pi/6)
% MJBOT: Model mj5208 brushless motor:
% Maximum RPM 7500 -> equivalent to 785.3982 rad/s
% Peak Torque: 1.7 Nm
% Torque Formula in Angular Motion: Torque = I x alpha, where I is moment
% of inertia and alpha is angular acceleration

% % Calculate Motor Model Max Angular Velocity in rad/s
% max_angular_vel = (2*pi*7500)/60;
% 
% % Convert from degrees to rad
% deg = 25;
% angle = deg*(pi/180)
% 
% % Formula for max angular velocity of a simple single pendulum model when
% % released from a specific angle q. This is for an approximation to
% % determine initial trajectory path states and will not generate and
% % accurate calculation. 
% g = 9.81;
% L1 = 0.195;       % Link 1 length (m)
% L2 = 0.215;       % Link 2 length (m) 
% 
% 
% q1 = 1.0399
% q2 = 0.8843;
% max_ang_vel_pen1 = sqrt((2*g/L1)*(1 - cos(q1)))     % rad/s
% max_ang_vel_pen2 = sqrt((2*g/L2)*(1 - cos(q2)));     % rad/s
% h1_max = (0.5*(L1^2)*(max_ang_vel_pen1^1))/g;
% h2_max = (0.5*(L2^2)*(max_ang_vel_pen2^2))/g;
% new_q1 = asin(h1_max/L1) + q1
% new_q2 = asin(h2_max/L2) + q2;
% 
% 
% % Hand Designed States
% % State format: state = [q1 q1_dot q2 q2_dot];
% % Convert states to csv file (potentially use excel) so that Rosh and
% % Daniel can use data
% state_0 = [0 0 0 0];     % initial state
% state_1 = [0 0 0. 0];
% state_3 = [0 0 0 0];
% state_4 = [0 0 0 0];
% state_5 = [0 0 0 0];
% state_6 = [0 0 0 0];
% state_7 = [0 0 0 0];
% state_8 = [pi 0 pi 0];    % final state

%% Calculation
% g = 9.81;
% L1 = 0.195;       % Link 1 length (m)
% L2 = 0.215;       % Link 2 length (m) 
% 
% q1 = 0.4363;
% q2 = 0.8843;
% max_ang_vel_pen1 = sqrt((2*g/L1)*(1 - cos(q1)));     % rad/s
% max_ang_vel_pen2 = sqrt((2*g/L2)*(1 - cos(q2)));     % rad/s
% h1_max = (0.5*(L1^2)*(max_ang_vel_pen1^2))/g;
% h2_max = (0.5*(L2^2)*(max_ang_vel_pen2^2))/g;
% new_q1 = asin(h1_max/L1) + 0.4363;
% new_q2 = asin(h2_max/L2) + 0.4363;
% 
% disp('Initial values:');
% disp(['q1: ', num2str(q1)]);
% disp(['max_ang_vel_pen1: ', num2str(max_ang_vel_pen1)]);
% disp(['new_q1: ', num2str(new_q1)]);
% 
% for i = 1:10
%     q1 = new_q1;
%     max_ang_vel_pen1 = sqrt((2*g/L1)*(1 - cos(q1)));     % rad/s
%     h1_max = (0.5*(L1^2)*(max_ang_vel_pen1^2))/g;
%     new_q1 = asin(h1_max/L1) + q1;
% 
%     disp(['Iteration ', num2str(i), ':']);
%     disp(['q1: ', num2str(q1)]);
%     disp(['max_ang_vel_pen1: ', num2str(max_ang_vel_pen1)]);
%     disp(['new_q1: ', num2str(new_q1)]);
% end

% %%
% g = 9.81;
% L1 = 0.195;       % Link 1 length (m)
% L2 = 0.215;       % Link 2 length (m) 
% q1 = 0.4363;
% q2 = 0.4363;
% maxIterations = 5;
% 
% % Preallocate arrays to store values
% q1_values = zeros(maxIterations+1, 1);
% max_ang_vel_pen1_values = zeros(maxIterations+1, 1);
% new_q1_values = zeros(maxIterations+1, 1);
% q2_values = zeros(maxIterations+1, 1);
% max_ang_vel_pen2_values = zeros(maxIterations+1, 1);
% new_q2_values = zeros(maxIterations+1, 1);
% 
% % Store initial values
% q1_values(1) = q1;
% max_ang_vel_pen1_values(1) = sqrt((2*g/L1)*(1 - cos(q1)));
% new_q1_values(1) = asin((0.5*(L1^2)*(max_ang_vel_pen1_values(1)^2))/g/L1) + q1;
% q2_values(1) = q2;
% max_ang_vel_pen2_values(1) = sqrt((2*g/L2)*(1 - cos(q2)));
% new_q2_values(1) = asin((0.5*(L2^2)*(max_ang_vel_pen2_values(1)^2))/g/L2) + q2;
% 
% for i = 1:maxIterations
%     % Update q1
%     q1 = new_q1_values(i);
%     max_ang_vel_pen1 = sqrt((2*g/L1)*(1 - cos(q1)));
%     h1_max = (0.5*(L1^2)*(max_ang_vel_pen1^2))/g;
%     new_q1 = asin(h1_max/L1) + q1;
% 
%     % Update q2
%     q2 = new_q2_values(i);
%     max_ang_vel_pen2 = sqrt((2*g/L2)*(1 - cos(q2)));
%     h2_max = (0.5*(L2^2)*(max_ang_vel_pen2^2))/g;
%     new_q2 = asin(h2_max/L2) + q2;
% 
%     % Store values
%     q1_values(i+1) = q1;
%     max_ang_vel_pen1_values(i+1) = max_ang_vel_pen1;
%     new_q1_values(i+1) = new_q1;
%     q2_values(i+1) = q2;
%     max_ang_vel_pen2_values(i+1) = max_ang_vel_pen2;
%     new_q2_values(i+1) = new_q2;
% end
% 
% % Display table
% disp('Iteration | q1         | max_ang_vel_pen1 | new_q1     | q2         | max_ang_vel_pen2 | new_q2');
% disp('---------------------------------------------------------------------------------------------');
% for i = 1:maxIterations+1
%     fprintf('%9d | %10.4f | %16.4f | %10.4f | %10.4f | %16.4f | %10.4f\n', ...
%             i-1, q1_values(i), max_ang_vel_pen1_values(i), new_q1_values(i), ...
%             q2_values(i), max_ang_vel_pen2_values(i), new_q2_values(i));
% end

% %%
g = 9.81;
m1 = 0.36;        % Link 1 mass (kg)
m2 = 0.21;        % Link 2 mass (kg)
L1 = 0.195;       % Link 1 length (m)
L2 = 0.215;       % Link 2 length (m)
q1 = 0.0902;
q2 = 0.1012;
maxIterations = 3;




% Preallocate arrays to store values
q1_values = zeros(maxIterations+1, 1);
max_ang_vel_pen1_values = zeros(maxIterations+1, 1);
new_q1_values = zeros(maxIterations+1, 1);
q2_values = zeros(maxIterations+1, 1);
max_ang_vel_pen2_values = zeros(maxIterations+1, 1);
new_q2_values = zeros(maxIterations+1, 1);

% Store initial values
q1_values(1) = q1;
max_ang_vel_pen1_values(1) = sqrt((2 * g / L1) * (1 - cos(q1))) * sqrt(m1 / (m1 * L1^2));
new_q1_values(1) = asin((0.5 * (L1^2) * (max_ang_vel_pen1_values(1)^2)) / g / L1) + q1;
q2_values(1) = q2;
max_ang_vel_pen2_values(1) = sqrt((2 * g / L2) * (1 - cos(q2))) * sqrt(m2 / (m2 * L2^2));
new_q2_values(1) = asin((0.5 * (L2^2) * (max_ang_vel_pen2_values(1)^2)) / g / L2) + q2;

for i = 1:maxIterations
    % Update q1
    q1 = new_q1_values(i);
    max_ang_vel_pen1 = sqrt((2 * g / L1) * (1 - cos(q1))) * sqrt(m1 / (m1 * L1^2));
    h1_max = (0.5 * (L1^2) * (max_ang_vel_pen1^2)) / g;
    new_q1 = asin(h1_max / L1) + q1;

    % Update q2
    q2 = new_q2_values(i);
    max_ang_vel_pen2 = sqrt((2 * g / L2) * (1 - cos(q2))) * sqrt(m2 / (m2 * L2^2));
    h2_max = (0.5 * (L2^2) * (max_ang_vel_pen2^2)) / g;
    new_q2 = asin(h2_max / L2) + q2;

    % Store values
    q1_values(i+1) = q1;
    max_ang_vel_pen1_values(i+1) = max_ang_vel_pen1;
    new_q1_values(i+1) = new_q1;
    q2_values(i+1) = q2;
    max_ang_vel_pen2_values(i+1) = max_ang_vel_pen2;
    new_q2_values(i+1) = new_q2;
end

% Display table
disp('Iteration | q1         | max_ang_vel_pen1 | new_q1     | q2         | max_ang_vel_pen2 | new_q2');
disp('---------------------------------------------------------------------------------------------');
for i = 1:maxIterations+1
    fprintf('%9d | %10.4f | %16.4f | %10.4f | %10.4f | %16.4f | %10.4f\n', ...
            i-1, q1_values(i), max_ang_vel_pen1_values(i), new_q1_values(i), ...
            q2_values(i), max_ang_vel_pen2_values(i), new_q2_values(i));
end


%% Hand Designed Justification 

