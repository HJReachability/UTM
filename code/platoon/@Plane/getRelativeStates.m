function relStates = getRelativeStates(obj, others)
% function relStates = getRelativeStates(obj, others)
% Finds state of other plane objects with respect to one plane
%
% Inputs:   obj - current plane object
%           others - other planes whose relative states relative
%                    to obj are to be found; this
%                    should be a n x 1 or 1 x n cell.
%
% Outputs:  relStates - relative state of each other plane
%
% Mahesh Vashishtha, 2015-10-27
others = checkVehiclesList(others, 'Plane');
relStates = zeros(length(others),4);
xp = obj.x;
for i=1:length(others)
  xe = others{i}.x;
  if size(xe, 1) ~= 4
    xe = xe';
  end
  if size(xp, 1) ~= 4
    xp = xp';
  end
  xr = xp -xe;
  % rotate position vector so it is correct relative to xe heading
  xr(1:2) = [cos(-xe(3)) -sin(-xe(3)); sin(-xe(3)) cos(-xe(3))] * xr(1:2);
  xr = xr';
  if xr(3) >= 2*pi
    xr(3) = xr(3) - 2*pi;
  end
  if xr(3) < 0
    xr(3) = xr(3) + 2*pi;
  end
  vel_comps = arrayfun(@getVelocity, [obj; others{i}],'un',0);
  xr(4) = norm(cell2mat(vel_comps(1,:)) - cell2mat(vel_comps(2,:)));
  relStates(i,:) = xr;
end
end