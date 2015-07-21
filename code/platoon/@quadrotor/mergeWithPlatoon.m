function u = mergeWithPlatoon(obj, p)
% function u = mergeWithPlatoon(obj, p)
%
% Computes the control used to merge into a platoon
% Free vehicles will merge into the first available slot
% Leader vehicles will take its entire platoon to combine with the target
% platoon
%
% Inputs:  obj  - quadrotor objects
%          p    - target platoon object to merge with
%
% Output:  u - control signal for the merge
%
% 2015-06-17, Mo
% Modified: Qie Hu, 2015-07-01
% Modified: Mo Chen, 2015-07-06

% Check if the current vehicle is a follower
switch obj.q
  case 'Free'
    % Check if platoon is full (should never have to check if outside logic is
    % correct
    if p.n+1 > p.nmax
      error('Platoon already full!')
    end
    
    % Index in the platoon to join
    idxJoin = find(~p.slotStatus, 1, 'first');
    
  case 'Leader'
    if p.loIdx + obj.p.loIdx > p.nmax
      error('Not enough slots at the back of the target platoon!')
    end
    
    % Index in the platoon to join
    idxJoin = p.loIdx + 1;
    
  case 'Follower'
    error(['Followers aren''t supposed to have the choice to join a' ...
      'platoon!'])
end

pdim = obj.pdim;
vdim = obj.vdim;

% Update join list
if isempty(obj.pJoin)
  % If current vehicle is not currently joining a platoon, then mark
  % p as the platoon to join and add to join list
  obj.idxJoin = idxJoin;
  p.slotStatus(obj.idxJoin) = -1;
  p.vJoin{obj.idxJoin} = obj;
  obj.pJoin = p;
  
else
  % Otherwise, empty previous marked platoon and remove from other
  % platoon's join list; also mark this platoon for joining
  if obj.pJoin ~= p
    obj.pJoin.slotStatus(obj.idxJoin) = 0;
    obj.pJoin.vJoin{obj.idxJoin} = [];
    
    obj.idxJoin = idxJoin;
    p.slotStatus(obj.idxJoin) = -1;
    p.vJoin{obj.idxJoin} = obj;
    obj.pJoin = p;
  end
end

% If vehicle has trailing vehicles inside the same platoon, recursively
% update their info too (could also do a loop, but this seems to work)
%
% This needs to be tested! Using visualizeVehicles.m is recommended.
vehicle = obj;
while vehicle.BQ ~= vehicle
  vehicle.BQ.idxJoin = vehicle.idxJoin + 1;
  p.slotStatus(vehicle.BQ.idxJoin) = -1;
  p.vJoin{vehicle.BQ.idxJoin} = vehicle.BQ;
  
  vehicle.BQ.pJoin = p;
  vehicle = vehicle.BQ;
end

% Parse target state
x = zeros(4,1);
x(vdim) = [0 0];

% Determine phantom position (First free position)
xPh = p.phantomPosition(obj.idxJoin);
x(pdim) = xPh - p.vehicles{1}.x(p.vehicles{1}.pdim);

% Time horizon for MPC
tsteps = 5;

if strcmp(obj.q, 'Free') || strcmp(obj.q, 'Leader')
  % If vehicle is free or a leader, then try to join the
  % platoon at the back
  
  % Reachable set from target state
  if isempty(obj.mergePlatoonV)
    [ datax, datay, g1, g2, tau ] = quad2D_joinHighwayPlatoon(x, 0);
    obj.mergePlatoonV.datax = datax;
    obj.mergePlatoonV.datay = datay;
    obj.mergePlatoonV.g1 = g1;
    obj.mergePlatoonV.g2 = g2;
    obj.mergePlatoonV.tau = tau;
    
  else
    datax = obj.mergePlatoonV.datax;
    datay = obj.mergePlatoonV.datay;
    g1 = obj.mergePlatoonV.g1;
    g2 = obj.mergePlatoonV.g2;
    tau = obj.mergePlatoonV.tau;
    
  end
  
  if abs(x-(obj.x-p.vehicles{1}.x))<=1.1*[g1.dx;g2.dx]
    % if relative state is within one grid point of target relative
    % state, roughly, then vehicle gets assimilated into platoon
    keyboard
    p.assimVehicle(obj);
    u = obj.followPlatoon;
    
  else
    % if relative state is not close enough, then try to head towards
    % the relative target state
    [valuex, gradx] = recon2x2D(tau, g1, datax, g2, ...
      datay, obj.x-p.vehicles{1}.x);
    
    if valuex <= 0
      % if vehicle is in the reachable set of target relative state,
      % use optimal control
      disp('Locked in')
      ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
      uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
      u = [ux; uy];
      
    else
      % if vehicle is not in the reachable set, move towards target
      % position in a straight line
      disp('Open-loop')
      
      % Path to target
      pathToOther = linpath(obj.x(pdim), xPh);
      u = obj.followPath(tsteps, pathToOther);
      
    end
  end
  
elseif strcmp(obj.q, 'Follower')
  error('Vehicle cannot be a follower!')
  
  % elseif strcmp(obj.q, 'Leader')
  %     error('Vehicle cannot be a leader!')
  
else
  error('Unknown mode!')
end

end
