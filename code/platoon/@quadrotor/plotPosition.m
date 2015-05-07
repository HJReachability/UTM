function plotPosition(obj, color)
% function plotPosition(obj)
%
% Plots the current state and the trajectory of the quadrotor


pdim = obj.pdim;
vdim = obj.vdim;


if isempty(obj.hpxpyhist) || ~isvalid(obj.hpxpyhist)
    if nargin<2
        obj.hpxpyhist = plot(obj.xhist(pdim(1),:), obj.xhist(pdim(2),:), ':'); hold on
    else
        obj.hpxpyhist = plot(obj.xhist(pdim(1),:), obj.xhist(pdim(2),:), ':', ...
            'color', color); hold on
    end
else
    obj.hpxpyhist.XData = obj.xhist(pdim(1),:); 
    obj.hpxpyhist.YData = obj.xhist(pdim(2),:); 
end

if isempty(obj.hpxpy) || ~isvalid(obj.hpxpy)
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
    obj.hpxpy.XData = obj.x(pdim(1)); 
    obj.hpxpy.YData = obj.x(pdim(2));
    obj.hpxpy.UData = obj.x(vdim(1));
    obj.hpxpy.VData = obj.x(vdim(2));
end

end