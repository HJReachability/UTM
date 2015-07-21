function assimVehicle(obj, vehicle)
% function assimVehicle(obj, vehicle)
% (as opposed to addVehicle, be careful!)
%
% Assimilates a vehicle that is trying to merge into the platoon. After
% this function is called, vehicle will be added to the platoon at position
% vehicle.idxJoin. If vehicle.idxJoin is not set, thenthe vehicle will be
% added to the first available position
%
% Inputs:   obj     - platoon object
%           vehicle - vehicle object to be assimilated
%
% Mo Chen, 2015-07-03
% Modified: Qie Hu, 2015-07-01
% Modified: Qie Hu, 2015-07-22

% If vehicle is 'Free', assimilating 1 vehicle
% If vehicle is 'Leader', assimilating a platoon of vehicles
    
% Check if the vehicle is a leader or free
if ~strcmp(vehicle.q, 'Free') && ~strcmp(vehicle.q, 'Leader')
  fprintf('Vehicle must be in free or leader mode! \n')
  return
end

% Check if the platoon is already full
if strcmp(vehicle.q, 'Free')
    % Vehicle is free
    if obj.loIdx >= obj.nmax
        fprintf('Platoon is already full! \n');
        return;
    end
else
    % Vehicle is a leader
    % Last occupied index of vehicle's platoon
    if obj.loIdx + vehicle.p.loIdx > obj.nmax
        fprintf('Platoon is already full! \n');
        return
    end
end

% Check to see if the vehicle is within highway width
[~, dist] = obj.hw.highwayPos(vehicle.x(vehicle.pdim));
if dist > obj.hw.width
  fprintf('Vehicle is too far from the highway! \n')
  return
end

% Find idxJoin 
if strcmp(vehicle.q, 'Free')
    % vehicle is free
    % join vehicle at first empty slot in obj
    if isempty(vehicle.idxJoin)
        idxJoin = find(cellfun('isempty', obj.vehicles), 1, 'first');
        vehicle.idxJoin = idxJoin;
    else
        idxJoin = vehicle.idxJoin;
    end
else
    % vehicle is a leader
    % join vehicle's platoon at the back of obj
    for i = find(vehicle.p.slotStatus == 1)'
        idxJoin = obj.loIdx + i;
        vehicle.p.vehicles{i}.idxJoin = idxJoin;
    end
end

xPh = obj.phantomPosition(idxJoin);

tol = 0.63; % Tolerance in distance to target state (g1.dx = g2.dx = [0.6250;0.6176])
x = zeros(4,1);
x(vehicle.vdim) = obj.hw.speed*obj.hw.ds;   % Target velocity is highway velocity
x(vehicle.pdim) = xPh;  % Target positin is phantom position
if abs(vehicle.x-x) >= 1.1*tol
    fprintf('Vehicle is too far from its target state! \n')
    return
end

% Add to platoon
if strcmp(vehicle.q, 'Free')
    % vehicle is free
    obj.updateVPropsJoin(vehicle);
else
    % vehicle is a leader
    obj.updatePPropsJoin(vehicle.p);
end

end