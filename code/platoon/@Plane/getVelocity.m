function vel = getVelocity(obj)
% vel = getPosition(obj)
% Finds the velocity (v_x,v_y) of a plane

vel = obj.speed * [cos(obj.x(3)); sin(obj.x(3))];

end