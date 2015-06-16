function addVehicle(obj, reachInfo, nextID)
% Adds a vehicle to the back of the platoon

% Check maximum allowed number of quadrotors
if obj.n >= obj.nmax
    error('Maximum number of vehicles reached! Cannot add more!')
end

% By default, the ID for the new quadrotor should be one after the maximum
% ID among the quadrotors in the current platoon

if nargin<3
    nextID = max([obj.vehicle.ID]) + 1;
end

dt = 0.1; % Hard coded... might get dangerous later on

if nargin<2 % If no reachInfo is specified, use default
    % Create quadrotor at the next phantom position
    x = obj.phantomPosition(obj, obj.n + 1);
    qr = quadrotor(nextID, dt, x);    
    
    % Update platoon fields
    obj.n = obj.n + 1;
    obj.vehicle = [obj.vehicle; qr];
else
    % Create quadrotor at the next phantom position
    x = obj.phantomPosition(obj, obj.n + 1);
    qr = quadrotor(nextID, dt, x, reachInfo);    
    
    % Update platoon fields
    obj.n = obj.n + 1;
    obj.vehicle = [obj.vehicle; qr];
end
end