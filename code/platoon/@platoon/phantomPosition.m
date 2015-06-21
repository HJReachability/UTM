function xPhantom = phantomPosition(obj, idx)
% function xPhantom = phantomPosition(obj, idx)
%
% Computes phantom position of a Follower
%
% Inputs:  obj      - current quadrotor object
%          idx      - position in platoon (leader has idx = 1)
% 
% Output:  xPhantom - phantom position
%
% Mo Chen, Qie Hu, Jaime Fisac, 2015-06-14

% Get leader vehicle and highway heading
Leader = obj.vehicle(1);

% Vehicle spacing is 3*2*sqrt(2) between each vehicle
spacing = 3*(2*sqrt(2));
xPhantom = Leader.x(Leader.pdim) - ...
    spacing * (idx-1) * obj.hw.ds; %* obj.platoon.followTime;
  
end