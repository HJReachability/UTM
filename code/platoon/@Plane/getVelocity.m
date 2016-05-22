function vel = getVelocity(obj)
% vel = getPosition(obj)
% Finds the velocity (v_x,v_y) of a plane

if isempty(obj.u)
  vel = obj.vrange(1) * [cos(obj.x(3)); sin(obj.x(3))];
else
  vel = obj.u(1) * [cos(obj.x(3)); sin(obj.x(3))];
end


end