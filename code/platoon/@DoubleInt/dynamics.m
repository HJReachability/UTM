function dx = dynamics(obj, t, x, u, ~, MIEdims)
% function dx = dynamics(t, x, u)
%     Dynamics of the double integrator

if nargin < 6
  MIEdims = 0;
end

if isnumeric(x)
  dx = [x(2); u];
end

if iscell(x)
  if isscalar(u)
    u = u*ones(size(x{1}));
  end
  dx = {x{2-MIEdims}; u};
end

end
