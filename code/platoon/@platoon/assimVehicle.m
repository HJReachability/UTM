function assimVehicle(obj, vehicle)
% function assimVehicle(obj, vehicle)
% (as opposed to addVehicle, be careful!)
%
% Assimilates a vehicle that is trying to merge into the platoon. After
% this function is called, vehicle will be added to the platoon at position
% vehicle.idx
%
% Inputs:   obj     - platoon object
%           vehicle - vehicle object to be assimilated
%
% Mo Chen, 2015-06-21
% Modified: Qie Hu, 2015-07-01

% Check if the platoon is already full
if obj.n >= obj.nmax
    fprintf('Platoon is already full! \n')
    return
end

% Check if the vehicle is a leader or free or emergencyLeader
if ~strcmp(vehicle.q, 'Free') && ~strcmp(vehicle.q, 'Leader') && ~strcmp(vehicle.q, 'EmergLeader')
    fprintf('Vehicle must be in free, leader or emergency leader mode! \n')
    return
end

% Check to see if the vehicle is within highway width
[~, dist] = obj.hw.highwayPos(vehicle.x(vehicle.pdim));
if dist > obj.hw.width
    fprintf('Vehicle is too far from the highway! \n')
    return
end

% Check to see if the vehicle is close to the phantom position
% (This only checks position... may want to check all states including
% velocity)
xPh = obj.phantomPosition(vehicle.idxJoin);
dtol = 2;                                  % Tolerance
if norm(xPh - vehicle.x(vehicle.pdim)) >= dtol
    fprintf('Vehicle is too far from its phantom position! \n')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modified

% Add to platoon
if strcmp(vehicle.q, 'Free')
    obj.updateVehicleProps(vehicle);
else
    % vehicle is a leader or emergLeader
    obj.updatePlatoonProps(vehicle.p);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end