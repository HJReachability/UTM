function theta = getHeading(obj)
% getHeading(obj)
% method of LinPath
%
% returns heading of highway in radians
theta = atan2(obj.ds(2), obj.ds(1));
end