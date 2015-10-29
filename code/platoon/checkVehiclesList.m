function vehicles = checkVehiclesList(vehicles, veh_type)
% function vehicles = checkVehiclesList(vehicles, veh_type)
% 
% Makes sure vehicles is a cell structure of vehicles all with the same
% type
%
% Inputs: vehicles - vehicle cell array
%         veh_type - expected type of vehicles
% Output: vehicles - vehicle cell array
% 
% Mo Chen, 2015-10-20

% Make sure vehicles is a cell structure; change it to a cell structure if
% it's a single vehicle
if ~iscell(vehicles)
  if length(vehicles) == 1
    vehicles = {vehicles};
  else
    error('others must be a vehicle object or a cell of vehicle objects!')
  end
end

% Make sure all other vehicles are of correct type
for i = 1:length(vehicles)
  if ~isa(vehicles{i}, veh_type)
    error(['All vehicles must be of type ' veh_type '!'])
  end
end

end