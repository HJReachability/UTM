function [p_rel, p_abs] = phantomPosition(obj, spacing, idx)
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

p_rel = - spacing * (idx-1) * obj.hw.ds;

if nargout > 1
  p_abs = obj.vehicles{1}.getPosition + p_rel;
end
end