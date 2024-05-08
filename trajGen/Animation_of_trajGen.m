% Calculate positions of pendulum ends
x1 = L1 * sin(q1_sim);
y1 = -L1 * cos(q1_sim);
x2 = x1 + L2 * sin(q1_sim + q2_sim);
y2 = y1 - L2 * cos(q1_sim + q2_sim);

% Create animation
figure;
for i = 1:length(t_sim)
    plot(x1(i), y1(i), 'ro', 'MarkerSize', m1 * 10, 'MarkerFaceColor', 'r');
    hold on;
    plot([0, x1(i)], [0, y1(i)], 'r-');
    plot(x2(i), y2(i), 'go', 'MarkerSize', m2 * 10, 'MarkerFaceColor', 'g');
    plot([x1(i), x2(i)], [y1(i), y2(i)], 'g-');
    hold off;
    axis equal;
    title(['Double Pendulum Animation (t = ', num2str(t_sim(i)), ')']);
    xlabel('X Position');
    ylabel('Y Position');
    xlim([-0.5, 0.5]); % Adjust xlim and ylim as needed
    ylim([-0.5, 0.5]);
    pause(0.01); 
end
