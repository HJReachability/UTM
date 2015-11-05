function [v, vhist] = getVelocity(obj)
% function [p, phist] = getPosition(obj)
%
% Returns the velocity and velocity history of a quardotor
%
% Mo Chen, 2015-10-20

v = obj.x(obj.vdim);
vhist = obj.xhist(obj.vdim,:);
end