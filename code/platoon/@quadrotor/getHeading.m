function heading = getHeading(obj)
% function heading = getHeading(obj)
%
% Computes the heading of the quadrotor
%
% Mo Chen, 2015-11-03

v = obj.getVelocity();
heading = atan2(v(2), v(1));
end