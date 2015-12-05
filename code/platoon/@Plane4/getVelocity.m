function vel = getVelocity(obj)
% vel = getPosition(obj)
% Finds the velocity (v_x,v_y) of a Plane4 object

  vel = obj.x(4) * [cos(obj.x(3)); sin(obj.x(3))];

end