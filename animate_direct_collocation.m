function animate_direct_collocation(d)
    % Extract states from design vector
    [ts, tauL, tauR, x, y, v, th, th_dot] = unpack_design_vector(d);

    subplot(2,2,1:4)
    for iter = 1:numel(ts) % Animation loop
        cla
        hold on
        title('robot animation')
        arrow = [0 0; cos(th(iter)) sin(th(iter))];

        % Plot the direction arrow
        plot(arrow(:, 1) + x(iter), arrow(:, 2) + y(iter), 'k-')
        % Plot the robot
        plot(x(iter), y(iter), 'ro')

        hold off
        axis equal
        axis([min(x)-1.5 max(x)+1.5 min(y)-1.5 max(y) + 1.25])
        xlabel('x position (m)')
        ylabel('y position (m)')
        % Forces the animation to plot mid-loop
        drawnow
    end
end