%% EQUATIONS OF MOTION - From Modern Robotics
function dqdt = double_pendulum_eqns(t, q, L1, L2, m1, m2, g, tau1, tau2)
    q1 = q(1);
    q2 = q(2);
    q1_dot = q(3);
    q2_dot = q(4);
    
    % Calculates the Mass Matrix
    M11 = m1 * L1^2 + m2 * (L1^2 + 2 * L1 * L2 * cos(q2) + L2^2);
    M12 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M21 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M22 = m2 * L2^2;
    M = [M11, M12; M21, M22];
    
    % Calculates the Coriolis and Centripetal Torques Vector 
    c11 = -m2 * L1 * L2 * sin(q2) * (2 * q1_dot * q2_dot + q2_dot^2);
    c21 = m2 * L1 * L2 * q1_dot^2 * sin(q2);
    c = [c11; c21];
    
    %Gravitational Torques Vector
    g_q11 = (m1 + m2) * L1 * g * cos(q1) + m2 * g * L2 * cos(q1 + q2);
    g_q21 = m2 * g * L2 * cos(q1 + q2);
    g_q = [g_q11; g_q21];
    
    % Overall Acceleration
    q_dot_dot = M \ ([-c(1) + tau1; -c(2) + tau2] - g_q);
    
    % dqdt contains the derivatives of the of the state variables, i.e., 
    % the angular velocities and accelerations of the joints. This vector 
    % represents the rate of change of the state variables with respect to 
    % time.
    dqdt = [q1_dot; q2_dot; q_dot_dot];
end
