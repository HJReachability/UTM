function diff = angle_diff(angle1, angle2)
% Computes the difference between two angles, taking into account wrapping
% at pi

diff = abs(angle2 - angle1);
if diff >= pi;
  diff = 2*pi - diff;
end
end