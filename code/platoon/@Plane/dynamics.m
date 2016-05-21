function dx = dynamics(obj, t, x, u)
% Dynamics of the Plane
%    \dot{x}_1 = v_x = x_4 * cos(x_3)
%    \dot{x}_2 = v_y = x_4 * sin(x_3)
%    \dot{x}_3 = u_1 = u_1
%
% For now, u_2 = 0 until we implement 4D reachable sets

dx = zeros(obj.nx, 1);

if numel(u) ~= obj.nu
  error('Incorrect number of control dimensions!')
end

if numel(obj.speed) == 1
  dx(1) = obj.speed * cos(x(3));
  dx(2) = obj.speed * sin(x(3));
  dx(3) = u;
  return
end

if isempty(obj.speed)
  dx(1) = u(1) * cos(x(3));
  dx(2) = u(1) * sin(x(3));
  dx(3) = u(2);
  return
end

error('Incorrect number of elements in obj.speed!')

end