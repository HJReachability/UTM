function u = followPlatoon(obj)
% function u = followPlatoon(obj)
%
% Follows the platoon that the current vehicle is in; the vehicle's current
% platoon is given by obj.p
%
% Input: obj - vehicle object
%
% Mo Chen, 2015-06-21

if ~strcmp(obj.q, 'Follower')
    error('Vehicle must be a follower!')
end

% Go to first available free position slot if needed
if find(~obj.p.vList, 1, 'first') < obj.idx
    obj.p.vList(obj.idx) = 0;
    obj.idx = find(~obj.p.vList, 1, 'first');
    obj.p.vList(obj.idx) = 1;
end

% Simple position and velocity feedback; gains could be tuned
k_p = 10;
k_v = 1;
u = obj.Leader.u ...
    + k_p*(obj.p.phantomPosition(obj.idx) - obj.x(obj.pdim))...
    + k_v*(obj.Leader.x(obj.vdim) - obj.x(obj.vdim));

% Acceleration limit
u = max(u, obj.uMin);
u = min(u, obj.uMax);

end % end function

