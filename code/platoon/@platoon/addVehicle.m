function addVehicle(obj, reachInfo, nextID)
% function addVehicle(obj, reachInfo, nextID)
%
% Adds a vehicle to the back of the platoon. A new vehicle will be created!
% (as opposed to assimVehicle(), careful!)
% This function is mainly for conveniently initializing a platoon. Also see
% popPlatoon.m
%
% Inputs: obj       - platoon object
%         reachInfo - reachability information to put into the vehicle
%                     being added
%         nextID    - the ID of the new vehicle being added
%
% Mo Chen, 2015-06-21
% Modified Qie Hu, 2015-06-29

% Check maximum allowed number of quadrotors
if obj.n >= obj.nmax
    warning('Maximum number of vehicles reached! Cannot add more!')
    return
end

% By default, the ID for the new quadrotor should be one after the maximum
% ID among the quadrotors in the current platoon

if nargin<3
    nextID = max([obj.vehicle.ID]) + 1;
end

dt = 0.1; % Hard coded... might get dangerous later on

% Create a quadrotor
x = zeros(4,1);
% Create quadrotor at the next phantom position at the same speed as
% the leader
x(obj.vehicle(1).pdim) = obj.phantomPosition(obj.n + 1);
x(obj.vehicle(1).vdim) = obj.vehicle(1).x(obj.vehicle(1).vdim);

if nargin<2 % If no reachInfo is specified, use default
    qr = quadrotor(nextID, dt, x);        
else
    qr = quadrotor(nextID, dt, x, reachInfo);    
end
qr.idx = nextID;

% Assimilate the quadrotor into the platoon
obj.assimVehicle(qr);
end