function [p, phist] = getPosition(obj)
% function [p, phist] = getPosition(obj)
%
% Returns the position and position history of a quardotor
%
% Mo Chen, 2015-10-20

p = obj.x(obj.pdim);
phist = obj.xhist(obj.pdim, :);
end