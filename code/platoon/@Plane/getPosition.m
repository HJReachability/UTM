function [p, phist] = getPosition(obj)
% pos = getPosition(obj)
% Gets the position (x,y) of a plane
p = obj.x(1:2);
phist = obj.xhist(1:2, :);
end