function removeVehicle(obj, vehicle)
% function u = removeVehicle(obj, vehicle)
%
% Remove vehicle from platoon without splitting the platoon
%
% Inputs: obj     - platoon object
%         vehicle - vehicle to be removed
%
% Qie Hu, 2015-07-01

% Check if vehicle is inside this platoon
if vehicle.p ~= obj
    fprintf('Cannot remove vehicle. Vehicle is not in this platoon!')
end

obj.updateVPropsAbandon(vehicle);

end