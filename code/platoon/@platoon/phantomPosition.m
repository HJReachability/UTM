function xPhantom = phantomPosition(obj, highwayPath, idx)
%
% Computes phantom position of a Follower
%
%

% if norm(obj.Leader.x(obj.vdim)) > 0
%     % If leader has non-zero velocity
%     % platoonHeading = leader velocity
%     platoonHeading = obj.Leader.x(obj.vdim)/norm(obj.Leader.x(obj.vdim));
% else
    % If leader is stopped, line up with highwayPath
    
% end

Leader = obj.vehicle{1};
platoonHeading = pathHeading(Leader, highwayPath);

xPhantom = Leader.x(Leader.pdim) - ...
    3*(2*sqrt(2)) * (idx-1) * platoonHeading; %* obj.platoon.followTime;
% xPhantom = obj.Leader.x(obj.pdim) - ...
%     (obj.platoon.followTime * norm(obj.Leader.x(obj.vdim))+2*2+1) * (obj.idx-1) * platoonHeading;

  
end

function nHeading = pathHeading(Leader,highwayPath)
% 
% Computes a normalized vector nHeading along direction of
% highwayPath at location of Leader.

N = 50;
s = linspace(0,1,N);
rpath = highwayPath(s);
p = Leader.x(Leader.pdim);
[~, idx] = min((rpath(1,:)-p(1)).^ 2 + (rpath(2,:)-p(2)).^2);
pDiff = highwayPath(s(min(50,idx+1))) - highwayPath(s(max(1,idx-1)));
nHeading = pDiff / norm(pDiff);


end