function vel = getVelocity(obj)
% vel = getPosition(obj)
% Finds the velocity (v_x,v_y) of a plane

if obj.nx == 3
  vel = obj.speed * [cos(obj.x(3)); sin(obj.x(3))];
else
  vel = obj.x(4) * [cos(obj.x(3)); sin(obj.x(3))];
end

end