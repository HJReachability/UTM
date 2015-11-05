function updateVPropsJoin(obj, vehicle)
% function updateVPropsJoin(obj, vehicle)
%
% Method of platoon class. Updates vehicle and platoon properties after adding a new vehicle has
% successfully added to the platoon
%
% Inputs: obj     - platoon object
%         vehicle - vehicle that is added
%
% Mo Chen, 2015-06-20
% Modified: Qie Hu, 2015-07-01


% Update number of vehicles in platoon
obj.n = obj.n + 1;

% Update last occupied slot index
obj.loIdx = max(obj.loIdx, vehicle.idxJoin);

% Confirm vehicle position in platoon
vehicle.idx = vehicle.idxJoin;
vehicle.idxJoin = [];

% Update vehicles pointers and join list pointers
obj.vehicles{vehicle.idx} = vehicle;
obj.vJoin{vehicle.idx} = [];

% Update vehicle list in platoon
obj.slotStatus(vehicle.idx) = 1;

% Empty platoon join pointer
vehicle.pJoin = [];

% Update pointers to and from the vehicle behind this vehicle 
BQ_idx = find(obj.slotStatus==1 & (1:obj.nmax)'>vehicle.idx, 1, 'first');
if isempty(BQ_idx)
  % If there are no vehicles behind this one, then the BQ pointer is to the
  % vehicle itself
  vehicle.BQ = vehicle;
else
  % Otherwise, update both the BQ pointer of this vehicle and the FQ
  % pointer of the vehicle behind it
  vehicle.BQ = obj.vehicles{BQ_idx};
  vehicle.BQ.FQ = vehicle;
end

% Update pointers to and from the vehicle in front of this vehicle
% (Should be guaranteed to exist, since the platoon must have at least one 
% vehicle to begin with)
FQ_idx = find(obj.slotStatus==1 & (1:obj.nmax)'<vehicle.idx, 1, 'last');
vehicle.FQ = obj.vehicles{FQ_idx};
vehicle.FQ.BQ = vehicle;

% Update other vehicle fields
vehicle.q = 'Follower';          % Mode
vehicle.p = obj;                 % Platoon pointer
vehicle.Leader = obj.vehicles{1}; % Leader pointer
% vehicle.mergePlatoonV = [];      % merge platoon value function

end