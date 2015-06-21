function addVehicleTest(obj, N)
% function addVehicleTest(obj, N)
%
% Tests the addVehicle method of platoon class
% 
% Inputs: obj - platoon object
%         N   - number of vehicles to add to the platoon
%
% Mo Chen, 2015-06-21

% Default number of times to add vehicles
if nargin < 2
    N = 5;
end

figure;

% Plot current platoon
obj.hw.hwPlot; hold on
color = lines(length(obj.vehicle));
for j = 1:length(obj.vehicle)
    obj.vehicle(j).plotPosition(color(j,:,:)); hold on
end
axis square
pause(1)

% Plot platoon after each vehicle add
for i = 2:N+1
    obj.addVehicle

    obj.hw.hwPlot; hold on
    color = lines(length(obj.vehicle));
    for j = 1:length(obj.vehicle)
        obj.vehicle(j).plotPosition(color(j,:,:)); hold on
    end
    axis square
    
    pause(1)
end