function trajDraft3
    % SECTION 1: Define the displacement, velocity and acceleration of double pendulum
    % masses
    syms theta_1(t) theta_2(t) L_1 L_2 m_1 m_2 g       % List the name of variables

    % Define symbolic velocities and accelerations
    syms theta_1_dot(t) theta_2_dot(t) theta_1_dot_dot(t) theta_2_dot_dot(t)

    % Defines the displacements of double pendulum in Cartesian Co-ordinates
    x_1 = L_1*sin(theta_1);
    y_1 = -L_1*cos(theta_1);
    x_2 = x_1 + L_2*sin(theta_2);
    y_2 = y_1 - L_2*cos(theta_2);

    % Finds the velocities through differentiation
    vx_1 = diff(x_1);
    vy_1 = diff(y_1);
    vx_2 = diff(x_2);
    vy_2 = diff(y_2);

    % Finds the accelerations through differentiation
    ax_1 = diff(vx_1);
    ay_1 = diff(vy_1);
    ax_2 = diff(vx_2);
    ay_2 = diff(vy_2);

    % SECTION 2: Define Equations of Motion 
    syms tau_1(t) tau_2(t)            % Torques acting on first and second pendulums

    % Evaluate torques acting on m_1:
    eqx_1 = m_1*ax_1 == tau_1*sin(theta_1) - tau_2*sin(theta_2);
    eqy_1 = m_1*ay_1 == tau_1*cos(theta_1) - tau_2*cos(theta_2) - m_1*g;

    % Evaluate torques acting on m_2:
    eqx_2 = m_2*ax_2 == -tau_2*sin(theta_2);
    eqy_2 = m_2*ay_2 == tau_2*cos(theta_2) - m_2*g;

    % SECTION 3: Solve System Equations
    % Define values 
    L_1 = 0.195;        % Link 1 length (m) (Motor shaft to motor shaft)
    L_2 = 0.215;        % Link 2 length (m) (Motor shaft to end of link)
    m_1 = 0.36;        % Link 1 mass (kg)
    m_2 = 0.21;        % Link 2 mass (kg)
    g = 9.8;        % gravity (m/s^2)

    % Convert equations to first-order differential equations
    eqn_1 = ode(eqx_1, eqy_1, theta_1, theta_2, theta_1_dot, theta_2_dot, L_1, L_2, m_1, m_2, g, tau_1);
    eqn_2 = ode(eqx_2, eqy_2, theta_1, theta_2, theta_1_dot, theta_2_dot, L_1, L_2, m_1, m_2, g, tau_2);

    % Solve the system of equations numerically
    sols = ode45(@(t,Y) [subs(eqn_1); subs(eqn_2)], [0 20], [0 0 0 0]);

    % SECTION 4: Create Animation of Oscillating Double Pendulum
    x_1 = @(t) L_1*sin(deval(sols,t,3));
    y_1 = @(t) -L_1*cos(deval(sols,t,3));
    x_2 = @(t) L_1*sin(deval(sols,t,3))+L_2*sin(deval(sols,t,1));
    y_2 = @(t) -L_1*cos(deval(sols,t,3))-L_2*cos(deval(sols,t,1));
    fanimator(@(t) plot(x_1(t),y_1(t),'ro','MarkerSize',m_1*10,'MarkerFaceColor','r'));
    axis equal;
    hold on;
    fanimator(@(t) plot([0 x_1(t)],[0 y_1(t)],'r-'));
    fanimator(@(t) plot(x_2(t),y_2(t),'go','MarkerSize',m_2*10,'MarkerFaceColor','g'));
    fanimator(@(t) plot([x_1(t) x_2(t)],[y_1(t) y_2(t)],'g-'));
    fanimator(@(t) text(-0.3,0.3,"Timer: "+num2str(t,2)));
    hold off;

    % SECTION 5: Define the ode function
    function eqn = ode(eqx, eqy, theta_1, theta_2, theta_1_dot, theta_2_dot, L_1, L_2, m_1, m_2, g, tau)
        eqn = vpasolve([eqx, eqy], [theta_1_dot_dot, theta_2_dot_dot]);
        eqn = odeToVectorField(eqn);
        eqn = matlabFunction(eqn, 'vars', {'t', 'Y'}, 'outputs', {'dYdt'});
        function res = ode_fun(t, Y)
            [dYdt] = eqn(t, Y);
            res = [Y(3); Y(4); dYdt(1); dYdt(2)];
        end
        eqn = @ode_fun;
    end
end
