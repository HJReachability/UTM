function u = followPlatoon(obj)
% function u = followPlatoon(obj)
%
% Follows the platoon that the current vehicle is in; the vehicle's current
% platoon is given by obj.p
%
% Input: obj - vehicle object
%
% Mo Chen, 2015-06-21

if ~strcmp(obj.q, 'Follower')
  error('Vehicle must be a follower!')
end

% Go to first available free position slot if needed
if find(~obj.p.slotStatus, 1, 'first') < obj.idx
  obj.p.slotStatus(obj.idx) = 0;
  obj.idx = find(~obj.p.slotStatus, 1, 'first');
  obj.p.vList(obj.idx) = 1;
  obj.mergePlatoonV = [];
end

% Parse target state
x = zeros(obj.nx, 1);

% Determine phantom position
xPh = obj.p.phantomPosition(obj.idx);

% Target state relative to leader
x(obj.pdim) = xPh - obj.Leader.x(obj.Leader.pdim);



if abs(x-(obj.x-obj.Leader.x))<=1.5%2.1*[g1.dx;g2.dx]
  disp('PID')
  % Simple position and velocity feedback; gains could be tuned
  k_p = 10;
  k_v = 1;

  if isempty(obj.Leader.u)
    lu = [0; 0];
  else
    lu = obj.Leader.u;
  end

  u = lu ...
    + k_p*(obj.p.phantomPosition(obj.idx) - obj.x(obj.pdim))...
    + k_v*(obj.Leader.x(obj.vdim) - obj.x(obj.vdim));

  % Acceleration limit
  u = max(u, obj.uMin);
  u = min(u, obj.uMax);
  obj.mergePlatoonV = [];
else
  disp('Catch-up')
  
  % Get value function
  [grids, datas, tau] = obj.computeV_relDyn(x);  
  
  % Catch up to platoon if too far away
  u = obj.computeCtrl_relDyn(obj.Leader.x, xPh, ...
    grids, datas, tau, obj.vMax);

end

end % end function

