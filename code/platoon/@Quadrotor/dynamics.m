function dx = dynamics(obj, t, x, u)
% function dx = dynamics(t, x, u)
%
% Dynamics of the quadrotor
%
% 2015-11-03

% A and B matrices
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; 1 0; 0 0; 0 1];

% Dynamics
dx = A * x + B * u;
end

