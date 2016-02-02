function vel = getVelocity(obj)
% vel = getPosition(obj)
% Finds the velocity (v_x,v_y) of a Plane4 object

vel = obj.getSpeed * [cos(obj.getHeading); sin(obj.getHeading)];

end