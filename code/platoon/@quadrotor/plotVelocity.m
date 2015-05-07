function plotVelocity(obj, color)
% function plotPosition(obj)
%
% Plots the current state and the trajectory of the quadrotor

if nargin<2, color = 'b'; end

vdim = obj.vdim;

if isempty(obj.hvxvyhist) || ~isvalid(obj.hvxvyhist)
    obj.hvxvyhist = plot(obj.xhist(vdim(1),:), obj.xhist(vdim(2),:), ':', 'color', color); hold on
else
    obj.hvxvyhist.XData = obj.xhist(vdim(1),:); 
    obj.hvxvyhist.YData = obj.xhist(vdim(2),:); 
end

if isempty(obj.hvxvy) || ~isvalid(obj.hvxvy)
    obj.hvxvy = plot(obj.x(vdim(1),:), obj.x(vdim(2),:), 'o', 'color', color); hold on
else
    obj.hvxvy.XData = obj.x(vdim(1)); 
    obj.hvxvy.YData = obj.x(vdim(2));
end

end