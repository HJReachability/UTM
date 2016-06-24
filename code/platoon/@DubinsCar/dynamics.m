function dx = dynamics(obj, t, x, u, ~, ~)
% Dynamics of the Plane
%    \dot{x}_1 = v * cos(x_3) + d1
%    \dot{x}_2 = v * sin(x_3) + d2
%    \dot{x}_3 = w + d3
%   Control: u = w;
%
% Mo Chen, 2016-06-08

if iscell(x)
  if nargin < 5
    d = {0; 0; 0};
  end
  dx = cell(obj.nx, 1);
  
  dx{1} = obj.speed * cos(x{3});
  dx{2} = obj.speed * sin(x{3});
  dx{3} = u;
else
  if nargin < 5
    d = [0; 0; 0];
  end
  dx = zeros(obj.nx, 1);
  
  dx(1) = obj.speed * cos(x(3));
  dx(2) = obj.speed * sin(x(3));
  dx(3) = u;
end


end