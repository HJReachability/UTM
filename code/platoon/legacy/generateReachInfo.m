function [reachInfo, safeV] = generateReachInfo( filename )
% function [reachInfo, safeV] = generateReachInfo( filename )
%
% Generates the reachable set information from a saved data file
%
% Mo Chen, 2015-07-14

if nargin < 1
%   filename = '../../data/quad3D.mat';
  filename = '../../data/quad2Dcollision.mat';
%    filename = '../../data/quad_safe_2x2D.mat';

end

load(filename)

safeV.tau = tau;
safeV.g = g;

if g.dim == 3
  safeV.dataS = dataS;
  safeV.dataC = dataC;
elseif g.dim == 2
  safeV.dataC = dataC;
elseif g.dim == 4
  safeV.dataC = dataC;
  safeV.grad  = grad;
else
  error('The safety reachable set must be 2, 3 or 4D!')
end

reachInfo.uMax = 1.7;
reachInfo.uMin = -1.7;
reachInfo.vMax = 5;
reachInfo.vMin = -5;

end