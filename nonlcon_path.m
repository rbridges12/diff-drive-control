% nonlcon.m

function [C, Ceq] = nonlcon_path(d, path)
    TauLim = 0.5;
    distLim = 0.5;
    
    R = 0.1; % Radius of wheels
    L = 0.5; % Distance between wheels
    m = 5; % Mass of robot
    D = 0; % COM from robot base frame origin
    J = 2; % Intertia about COM
    b = 1; % ?

    % Extract states from design vector
    N = ((numel(d)-1)/7);
    tauL = d(1:N);
    tauR = d(N+1:2*N);
    x = d(2*N+1:3*N);
    y = d(3*N+1:4*N);
    v = d(4*N+1:5*N);
    th = d(5*N+1:6*N);
    th_dot = d(6*N+1:7*N);
    tf = d(end);
    ts = tf/(N-1);

    % Estimate
    dx_hat = x(2:end) - x(1:end-1);
    dy_hat = y(2:end) - y(1:end-1);
    dv_hat = v(2:end) - v(1:end-1);
    dth_hat = th(2:end) - th(1:end-1);
    dth_dot_hat = th_dot(2:end) - th_dot(1:end-1);

    % targets
    dx_target = v(1:end-1).*cos(th(1:end-1))*ts;
    dy_target = v(1:end-1).*sin(th(1:end-1))*ts;
    dth_target = th_dot(1:end-1)*ts;

    % targets v2
    qs = [x(1:end-1)'; y(1:end-1)'; v(1:end-1)'; th(1:end-1)'; th_dot(1:end-1)'];
    us = [tauL(1:end-1)'; tauR(1:end-1)'];
    qdots = odefun_torque(qs,us,L,m,D,J,R,b);
    dv_target = qdots(3,:)';
    dth_dot_target = qdots(5,:)';

    % Initial and final constraints
    q0_target = [path(1,1);path(1,2);0;0;0];
    qf_target = [path(end,1);path(end,2);0;0;0];
    q0 = [x(1); y(1); v(1); th(1); th_dot(1)];
    qf = [x(end); y(end); v(end); th(end); th_dot(end)];
    Ceq = [q0 - q0_target; qf - qf_target];

    % Combine defect constraints
    Ceq = [Ceq; dx_hat - dx_target];
    Ceq = [Ceq; dy_hat - dy_target];
    Ceq = [Ceq; dv_hat - dv_target];
    Ceq = [Ceq; dth_hat - dth_target];
    Ceq = [Ceq; dth_dot_hat - dth_dot_target];

    % Inequality constraints
    C =[tauL - TauLim; tauR - TauLim; -tauR - TauLim; -tauL - TauLim; -tf];
end



