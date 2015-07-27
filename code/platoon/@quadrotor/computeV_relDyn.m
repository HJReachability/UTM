function [ datax, datay, g1, g2, tau ] = computeV_relDyn(obj, x)
% Reachable set from target state
if isempty(obj.mergePlatoonV)
  [ datax, datay, g1, g2, tau ] = quad2D_joinHighwayPlatoon(x, 0);
  obj.mergePlatoonV.datax = datax;
  obj.mergePlatoonV.datay = datay;
  obj.mergePlatoonV.g1 = g1;
  obj.mergePlatoonV.g2 = g2;
  obj.mergePlatoonV.tau = tau;
  
else
  datax = obj.mergePlatoonV.datax;
  datay = obj.mergePlatoonV.datay;
  g1 = obj.mergePlatoonV.g1;
  g2 = obj.mergePlatoonV.g2;
  tau = obj.mergePlatoonV.tau;
  
end
end