function updateProps1(obj, vehicle)
% function updateProps1(obj, vehicle)
%
% Method of platoon class. Updates vehicle and platoon properties after adding a new vehicle has
% successfully added to the platoon
%
% Inputs: obj     - platoon object
%         vehicle - vehicle that is added
%
% Mo Chen, 2015-06-20

% Update number of vehicles in platoon
obj.n = obj.n + 1;

% Update vehicle pointer list
if vehicle.idx == obj.vehicle(end).idx
    % If the new vehicle's position index already exists, something went
    % terribly wrong...
    error('The vehicle joining the platoon is already in the platoon?')
    
else
    % Otherwise, recreate vehicle list by inserting vehicle
    % after the last vehicle in the current list with position index
    % smaller than the current vehicle to be added
    l_idx = find([obj.vehicle.idx] < vehicle.idx, 1, 'last');
    obj.vehicle = [obj.vehicle(1:l_idx); vehicle; obj.vehicle(l_idx+1:end)];
end

% Update vehicle list in platoon
obj.vList(vehicle.idx) = 1;
vehicle.pJoin = [];

% Update pointers to front and behind vehicles for this vehicle and the
% vehicle in front
obj.vehicle(l_idx).BQ = vehicle; % Point previous trailer
vehicle.FQ = obj.vehicle(l_idx);

% If there's a vehicle behind, update front and behind pointers for this
% vehicle and the vehicle behind
c_idx = l_idx + 1;
if ~isempty(obj.vehicle(c_idx+1:end))
    obj.vehicle(c_idx+1).FQ = vehicle;
    vehicle.BQ = obj.vehicle(c_idx+1);
end

% Update current vehicle position index
vehicle.idx = c_idx;

% Update other vehicle fields
obj.vehicle(end).q = 'Follower';          % Mode
obj.vehicle(end).p = obj;                 % Platoon pointer
obj.vehicle(end).Leader = obj.vehicle(1); % Leader pointer
obj.vehicle(end).mergePlatoonV = [];      % merge platoon value function
end