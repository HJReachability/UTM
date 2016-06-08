function dx = dynamics(obj, t, x, u, d)
% Dynamics of the Plane
%    \dot{x}_1 = v * cos(x_3) + d1
%    \dot{x}_2 = v * sin(x_3) + d2
%    \dot{x}_3 = w + d3
%   Control: u = w;
%
% Mo Chen, 2016-06-08

if nargin < 5
  d = [0; 0; 0];
end

if numel(u) ~= obj.nu
  error('Incorrect number of control dimensions!')
end

dx = zeros(obj.nx, 1);

% Kinematic plane (speed can be changed instantly)
dx(1) = obj.speed * cos(x(3)) + d(1);
dx(2) = obj.speed * sin(x(3)) + d(2);
dx(3) = u + d(3);

end