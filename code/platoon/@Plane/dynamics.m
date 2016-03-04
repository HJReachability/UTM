function dx = dynamics(obj, t, x, u)
% Dynamics of the Plane
%    \dot{x}_1 = v_x = x_4 * cos(x_3)
%    \dot{x}_2 = v_y = x_4 * sin(x_3)
%    \dot{x}_3 = u_1 = u_1
%
% For now, u_2 = 0 until we implement 4D reachable sets

dx = zeros(obj.nx, 1);

dx(1) = obj.speed * cos(x(3));
dx(2) = obj.speed * sin(x(3));
dx(3) = u;

end