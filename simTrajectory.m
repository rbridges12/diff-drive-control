function [tout, yout] = simTrajectory(x)
    % TODO fill out
    R = 2; % Radius of each wheel
    L = 5; % Distance between wheels
    tf = 5;

    % have x = angular velocity spline? x = [vl1, ... vln, vr1, ... vrn, tf];
    interp_vel_l = @(t) interp1(linspace(0, tf, numel(x)/2), x(1:end/2), t);
    interp_vel_r = @(t) interp1(linspace(0, tf, numel(x)/2), x(end/2+1:end), t);

    % v = R(vr + vl)/2
    % w = R(vr - vl)/2

    % x_dot, y_dot, theta_dot
    % TODO: We only use theta here?
    odefun = @(t, y) [R/2*(interp_vel_r(t) + interp_vel_l(t)) * cos(y(3)); 
        R/2*(interp_vel_r(t) + interp_vel_l(t)) * sin(y(3));
        R/L * (interp_vel_r(t) - interp_vel_l(t))];
    [tout, yout] = ode45(odefun, [0, tf], [0;0;pi/4]);


end