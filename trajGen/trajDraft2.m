
% SECTION 1: Define the displacement, velocity and acceleration of double pendulum
% masses
syms theta_1(t) theta_2(t) L_1 L_2 m_1 m_2 g        % List the name of variables

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
syms T_1 T_2            % Tension of first rod (T_1) Tension of second rod (T_2)

% Evaluate forces acting on m_1:
eqx_1 = m_1*ax_1(t) == -T_1*sin(theta_1(t)) + T_2*sin(theta_2(t));
eqy_1 = m_1*ay_1(t) == T_1*cos(theta_1(t)) - T_2*cos(theta_2(t)) - m_1*g;

% Evaluate forces acting on m_2:
eqx_2 = m_2*ax_2(t) == -T_2*sin(theta_2(t));
eqy_2 = m_2*ay_2(t) == T_2*cos(theta_2(t)) - m_2*g;

% SECTION 3: Evaluate Forces and Reduce System Equations
% There are 4 unknowns in the equations of motion: theta_1, theta_2, T_1,
% T_2
% Evaluate the 2 unknowns T_1 and T_2:
Tension = solve([eqx_1 eqy_1],[T_1 T_2]);

% Substitute the solutions for T_1 and T_2 into eqx_2 and eqy_2
% These 2 reduce equations describe the pendulum motion
eqRed_1 = subs(eqx_2,[T_1 T_2],[Tension.T_1 Tension.T_2]);
eqRed_2 = subs(eqy_2,[T_1 T_2],[Tension.T_1 Tension.T_2]);


% SECTION 4: Solve System Equations

% Define values 
L_1 = 1;        % arm 1 length (m)
L_2 = 1;        % arm 2 length (m)
m_1 = 1;        % arm 1 mass (kg)
m_2 = 1;        % arm 2 mass (kg)
g = 9.8;        % gravity (m/s^2)

% Substitute values into reduced equations
eqn_1 = subs(eqRed_1);
eqn_2 = subs(eqRed_2);

% Convert to first-order differential equations
% Elements of the vector v represent the first-order differential
% equations that are equal to the time derivative of the elements of S. The
% elements of S are the state variables theta_2, dtheta_2/dt, theta_1,
% dtheta_1/dt. These state variables decribe the angular displacements and
% velocities of the double pendulum.
[V,S] = odeToVectorField(eqn_1,eqn_2);

% Convert first order-differential equations to MATLAB function with the
% handle M.
M = matlabFunction(V,'vars',{'t','Y'});

% Define the inital conditions of the state variables 
initCond = [0 0 0 0];

% Use ode45 to sol be the state variables. The solutions are a function of
% time within the interval [0, 10]
sols = ode45(M,[0 10],initCond);

% Plot solutions of state variables
plot(sols.x,sols.y)
legend('\theta_2','d\theta_2/dt','\theta_1','d\theta_1/dt')
title('Solutions of State Variables')
xlabel('Time (s)')
ylabel('Solutions (rad or rad/s)')

%SECTION 5: Create Animation of Oscillating Double Pendulum
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
