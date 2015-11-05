function pPlot(obj, cMap, hwcolor)
% function pPlot(obj, cMap, hwcolor)
%
% Plots a the positions of the members of the platoon, as well as the
% highway the platoon is on.
%
% Inputs: obj     - platoon object
%         cMap    - color map for the vehicles
%         hwcolor - color of the highway
% 
% Mo Chen, 2015-07-21

% Default colormap for vehicles
if nargin<2, cMap = lines(obj.n); end

% Default color for highway
if nargin<3, hwcolor = 'k'; end

% Plot highway
obj.hw.hwPlot(hwcolor); hold on

% Plot the vehicles
for i = 1:obj.n
    color = cMap(i,:);
    obj.vehicles{i}.plotPosition(color); hold on
end

end