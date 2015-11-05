function addVehicle(obj, nextID, reachInfo)
% function addVehicle(obj, nextID, reachInfo)
%
% Adds a vehicle to the back of the platoon. A new vehicle will be created!
% (as opposed to assimVehicle(), careful!)
%
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

if nargin<2
    reachInfo = generateReachInfo();
end

% By default, the ID for the new quadrotor should be one after the maximum
% ID among the quadrotors in the current platoon
if nargin<1
    nextID = max([obj.vehicle.ID]) + 1;
end

% Create quadrotor at the next phantom position at the same speed as
% the leader
% dt = 0.1; % Hard coded... might get dangerous later on

if nargin<3 % If no reachInfo is specified, use default
  qr = quadrotor(nextID, zeros(4,1));
else
  qr = quadrotor(nextID, zeros(4,1), reachInfo);
end 

% Specify desired state
qr.x(qr.pdim) = obj.phantomPosition(obj.n + 1);
qr.x(qr.vdim) = obj.vehicles{1}.x(obj.vehicles{1}.vdim);

% Assimilate the quadrotor into the platoon
obj.assimVehicle(qr);
end