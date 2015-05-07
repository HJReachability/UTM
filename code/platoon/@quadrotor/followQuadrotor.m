function u = followQuadrotor(obj, other, tsteps)
% function u = followQuadrotor(obj, other, tf)
%
% Follows the "other" quadrotor

minDist = min((other.xhist(1,:) - obj.x(1)).^2 + (other.xhist(2,:) - obj.x(2)).^2);

rpath = @(s) [(1-s)*obj.x(1) + s*other.x(1); (1-s)*obj.x(3) + s*other.x(3)];
u = obj.followPath(tsteps, rpath);

end