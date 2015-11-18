function xPhantom = phantomPosition(obj, spacing, idx)
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
% Modified: Mo Chen, 2015-07-06

% Get leader vehicle and highway heading
Leader = obj.vehicles{1};

xPhantom = Leader.x(Leader.pdim) - spacing * (idx-1) * obj.hw.ds;
  
end