function addPlatoon(obj, hw, p)
% function addPlatoon(obj, hw, p)
%
% Adds the platoon p to the highway hw
%
% Mo Chen, 2015-11-03

hw.ps = {hw.ps; p};

obj.addActiveAgents(p);

end