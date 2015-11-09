function [v, vhist] = getVelocity(obj)
v = obj.x(2);
vhist = obj.xhist(2,:);
end