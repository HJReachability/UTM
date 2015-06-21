function hwPlot(obj, color)
% function hwPlot(obj)
%
% Plots the highway
%
% Inputs: color - color to plot the highway in
% 
% Mo Chen, 2015-06-21

if nargin<2, color = 'k'; end

pts = obj.fn([0 1]);
obj.h = plot(pts(1,:), pts(2,:), '--', 'color', color);

end