function [tout, yout] = sim_trajectory_single_shooting(x)
    R = 0.1; % Radius of each wheel
    L = 0.5; % Distance between wheels
    M = 5;
    d = 0;
    J = 2;
    b = 1;
    tf = 5;

    % have x = angular velocity spline? x = [vl1, ... vln, vr1, ... vrn, tf];
    interp_t_l = @(t) interp1(linspace(0, tf, numel(x)/2), x(1:end/2), t);
    interp_t_r = @(t) interp1(linspace(0, tf, numel(x)/2), x(end/2+1:end), t);

    odefun = @(t, y) odefun_torque(y, [interp_t_l(t); interp_t_r(t)], L, M, d, J, R, b);
    [tout, yout] = ode45(odefun, [0, tf], [0;0;0;0;0]);


end