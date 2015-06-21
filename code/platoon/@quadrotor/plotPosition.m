function plotPosition(obj, color)
% function plotPosition(obj, color)
%
% Plots the current state and the trajectory of the quadrotor
% 
% Inputs: obj   - vehicle object
%         color - color for plotting
%
% Mo Chen, 2015-06-21

pdim = obj.pdim;
vdim = obj.vdim;

% Plot position trajectory
if isempty(obj.hpxpyhist) || ~isvalid(obj.hpxpyhist)
    % If no graphics handle has been created, create it. Use custom color
    % if provided.
    if nargin<2
        obj.hpxpyhist = plot(obj.xhist(pdim(1),:), obj.xhist(pdim(2),:), ':'); hold on
    else
        obj.hpxpyhist = plot(obj.xhist(pdim(1),:), obj.xhist(pdim(2),:), ':', ...
            'color', color); hold on
    end
else
    % Otherwise, simply update the graphics handles
    obj.hpxpyhist.XData = obj.xhist(pdim(1),:); 
    obj.hpxpyhist.YData = obj.xhist(pdim(2),:); 
end

% Plot current position and velocity using an arrow
if isempty(obj.hpxpy) || ~isvalid(obj.hpxpy)
    % If no graphics handle has been created, create it with the specified
    % color. Use default color if no color is provided.
    if nargin<2
        obj.hpxpy = quiver(obj.x(pdim(1)), obj.x(pdim(2)), ...
            obj.x(vdim(1)), obj.x(vdim(2)), 'o', 'MaxHeadSize', 2, ...
                                        'ShowArrowHead', 'on'); hold on;
    else
        obj.hpxpy = quiver(obj.x(pdim(1),:), obj.x(pdim(2),:), ...
        obj.x(vdim(1)), obj.x(vdim(2)), 'o', 'MaxHeadSize', 2, ... 
                           'ShowArrowHead', 'on', 'color', color); hold on
    end
    
    obj.hpxpy.Color = obj.hpxpyhist.Color;
    obj.hpxpy.MarkerFaceColor = obj.hpxpyhist.Color;
    obj.hpxpy.MarkerSize = 6;
else
    % Otherwise, simply update graphics handles
    obj.hpxpy.XData = obj.x(pdim(1)); 
    obj.hpxpy.YData = obj.x(pdim(2));
    obj.hpxpy.UData = obj.x(vdim(1));
    obj.hpxpy.VData = obj.x(vdim(2));
end

end