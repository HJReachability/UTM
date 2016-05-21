function pVisualize(obj, f)
% function pVisualize(obj)
% Visualizes platoon
% f - figure handle

if nargin<2
    f = figure;
end

visualizeVehicles(obj.vehicle, f)

end
