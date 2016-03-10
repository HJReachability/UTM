function vel = getVelocity(obj)
% vel = getPosition(obj)
% Finds the velocity (v_x,v_y) of a plane

if isempty(obj.speed)
  if isempty(obj.u)
    vel = [1e-3; 0];
  else
    vel = obj.u(1) * [cos(obj.x(3)); sin(obj.x(3))];
  end
  return
end

if numel(obj.speed) == 1
  vel = obj.speed * [cos(obj.x(3)); sin(obj.x(3))];
  return
end

end