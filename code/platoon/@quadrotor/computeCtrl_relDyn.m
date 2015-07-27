function u = computeCtrl_relDyn(obj, refState, mvTarget, ... 
  datax, datay, g1, g2, tau, speed)

% if relative state is not close enough, then try to head towards
% the relative target state
[valuex, gradx] = recon2x2D(tau, g1, datax, g2, ...
  datay, obj.x-refState);

if valuex <= 0
  % if vehicle is in the reachable set of target relative state,
  % use optimal control
  disp('Locked in')
  ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
  uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
  u = [ux; uy];
  
else
  % if vehicle is not in the reachable set, move towards target
  % position in a straight line
  disp('Open-loop')
  
  % Path to target
  pathToOther = linpath(obj.x(obj.pdim), mvTarget);
  tsteps = 5;
  if speed~=3
    LQROn = 0;
  else
    LQROn = 1;
  end
  u = obj.followPath(tsteps, pathToOther, speed, LQROn);
  
end

end