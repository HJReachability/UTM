function [reachInfo, safeV] = generateReachInfo( filename )

if nargin < 1
    filename = 'quad3D.mat';
end

load(filename)

safeV.tau = tau;
safeV.dataS = dataS;
safeV.dataC = dataC;
safeV.g = g;

reachInfo.uMax = 1.7;
reachInfo.uMin = -1.7;
reachInfo.vMax = 5;
reachInfo.vMin = -5;

end




