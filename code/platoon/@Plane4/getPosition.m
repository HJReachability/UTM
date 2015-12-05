function [p, phist] = getPosition(obj)
% pos = getPosition(obj)
% Gets the position (x,y) and position history of a Plane4 object
p = obj.x(1:2);
phist = obj.xhist(1:2, :);
end