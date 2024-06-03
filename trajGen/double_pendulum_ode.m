function dydt = double_pendulum_ode(t, y, q1_desired, q2_desired, q1_dot_desired, q2_dot_desired, t_sim, m1, m2, L1, L2, g)
    % Interpolate the desired positions at time t
    q1 = interp1(t_sim, q1_desired, t, 'linear');
    q2 = interp1(t_sim, q2_desired, t, 'linear');

    % Compute the derivatives of positions
    dydt = zeros(4, 1);
    dydt(1) = y(2); % q1_dot
    dydt(2) = y(4); % q2_dot

    % Compute the derivatives of velocities (accelerations)
    % Equations of motion
    M11 = m1 * L1^2 + m2 * (L1^2 + 2 * L1 * L2 * cos(q2) + L2^2);
    M12 = m2 * (L1 * L2 * cos(q2) + L2^2);
    M21 = M12;
    M22 = m2 * L2^2;
    M = [M11, M12; M21, M22];
    
    c11 = -m2 * L1 * L2 * sin(q2) * (2 * y(2) * y(4) + y(4)^2);
    c21 = m2 * L1 * L2 * y(2)^2 * sin(q2);
    c = [c11; c21];
    
    g_q11 = (m1 + m2) * L1 * g * cos(q1) + m2 * g * L2 * cos(q1 + q2);
    g_q21 = m2 * g * L2 * cos(q1 + q2);
    g_q = [g_q11; g_q21];
    
    % Compute accelerations
    q_dot_dot = M \ ([-c(1); -c(2)] - g_q);
    
    % Assign accelerations to the output
    dydt(3) = q_dot_dot(1); % q1_dot_dot
    dydt(4) = q_dot_dot(2); % q2_dot_dot
end
