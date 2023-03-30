% Extract states from design vector
function [ts,tauL,tauR,x,y,v,th,th_dot] = unpack_design_vector(d)
    N = ((numel(d)-1)/7);
    tauL = d(1:N);
    tauR = d(N+1:2*N);
    x = d(2*N+1:3*N);
    y = d(3*N+1:4*N);
    v = d(4*N+1:5*N);
    th = d(5*N+1:6*N);
    th_dot = d(6*N+1:7*N);
    tf = d(end);
    ts = linspace(0, tf, N);
end
