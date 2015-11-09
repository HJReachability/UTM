function [p, phist] = getPosition(obj)
p = obj.x(1);
phist = obj.xhist(1,:);
end