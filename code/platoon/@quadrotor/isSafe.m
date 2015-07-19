function [safe, uSafe, valuex] = ...
  isSafe(obj, other, safeV, t)
% function [safe, uSafe, valuex] = isSafe(obj, other, safeV, t)
%
% Checks whether this vehicle is safe within a time horizon of t with
% respect to the other vehicle
%
% Inputs:  obj   - this vehicle
%          other - other vehicle
%          t     - time horizon
%
% Outputs: safe  - boolean variable indicating whether the relative state
%                  is safe
%          uSafe - the optimal safe controller
%          valuex -the value of levelset function
% Dynamics:
%    \dot{p}_{x,r}   = v_{x,r}
%    \dot{v}_{x,r}   = obj.u_x - other.u_x
%    \dot{v}_{x,obj} = obj.u_x
%    \dot{p}_{y,r}   = v_{y,r}
%    \dot{v}_{y,r}   = obj.u_y - other.u_y
%    \dot{v}_{y,obj} = obj.u_y
%
% Mo Chen, 2015-05-23
% Modified: Qie Hu, 2015-07-01
% Modified: Mo Chen. 2015-07-14

% Unpack reachable set
tau = safeV.tau;
g = safeV.g;

if safeV.g.dim == 3
  dataC = safeV.dataC;
  dataS = safeV.dataS;
  
  
  % States in 6D reachable set
  xr = obj.x(1) - other.x(1);
  vxr = obj.x(2) - other.x(2);
  vx = obj.x(2);
  yr = obj.x(3) - other.x(3);
  vyr = obj.x(4) - other.x(4);
  vy = obj.x(4);
  x = [xr vxr vx yr vyr vy];
  
  % If the state is more than a grid point away from the computation domain,
  % then the relative system is safe
  if any(x' <= [g.min+g.dx; g.min+g.dx])
    safe = 1;
    uSafe = [0; 0];
    valueCx = max(dataC(:));
    valuex = valueCx;
    return
  end


  if any(x' >= [g.max-g.dx; g.max-g.dx])
    safe = 1;
    uSafe = [0; 0];
    valueCx = max(dataC(:));
    valuex = valueCx;
    return
  end
  
  t = obj.tauInt+0.5; % Migrate towards a single safety time window?
  
  % Compute value at current state
  % Value according to collision criterion
  valueCx = recon2x3D(tau, g, dataC, g, dataC, x, t);
  
  % Value according to velocity limit criterion
  ind = min(length(tau), find(tau<=t,1,'last')+1);
  valueSxx = eval_u(g, dataS(:,:,:,ind), x(1:3));
  valueSxy = eval_u(g, dataS(:,:,:,ind), x(4:6));
  valueSx = min(valueSxx, valueSxy);
  
  % Minimum value is safety value
  valuex = min(valueCx, valueSx);
  
  % Is the value safe?
  if valuex <= 0, safe = false;
  else            safe = true;
  end


  % Compute gradient of V(t,x) where t is the first t such that V(t,x) <= 0
  [~, ~, g6D, valueC, ~, ind] = recon2x3D(tau, g, dataC, g, dataC, x, t);
  valueSSx = eval_u(g, dataS(:,:,:,ind), [g6D.xs{1}(:) g6D.xs{2}(:) g6D.xs{3}(:)]);
  valueSSy = eval_u(g, dataS(:,:,:,ind), [g6D.xs{4}(:) g6D.xs{5}(:) g6D.xs{6}(:)]);
  valueS = min(valueSSx, valueSSy);
  valueS = reshape(valueS, g6D.shape);
  value = min(valueC, valueS);
  
  % Convert to signed distance function for fair comparison of the two
  % criteria (using default [medium] accuracy for faster speed)
  % Get rid of this
  valuesd = signedDistanceIterative(g6D, value,'low'); %AKA modified 7/12/2015
  
  % Now we can finally read off gradient (Using first order derivative for
  % speed)
  gradsd = extractCostates(g6D, valuesd,'low'); %AKA modified 7/12/2015
  gradx = calculateCostate(g6D, gradsd, x);
  
  % Compute optimal safe controller
  uSafe = [(gradx(2)+gradx(3)>=0)*obj.uMax + (gradx(2)+gradx(3)<0)*obj.uMin; ...
    (gradx(5)+gradx(6)>=0)*obj.uMax + (gradx(5)+gradx(6)<0)*obj.uMin];
  
elseif safeV.g.dim == 2
  dataC = safeV.dataC;
  
  % States in 6D reachable set
  xr = obj.x(1) - other.x(1);
  vxr = obj.x(2) - other.x(2);
  yr = obj.x(3) - other.x(3);
  vyr = obj.x(4) - other.x(4);
  x = [xr vxr yr vyr];
  
  % If the state is more than a grid point away from the computation domain,
  % then the relative system is safe
  if any(x' <= [g.min+g.dx; g.min+g.dx])
    safe = 1;
    uSafe = [0; 0];
    valueCx = max(dataC(:));
    valuex = valueCx;
    return
  end
  
  if any(x' >= [g.max-g.dx; g.max-g.dx])
    safe = 1;
    uSafe = [0; 0];
    valueCx = max(dataC(:));
    valuex = valueCx;
    return
  end
  
  t = obj.tauInt+0.5; % Migrate towards a single safety time window?

    % Compute value at current state
  % Value according to collision criterion
  [valuex, gradx] = recon2x2D(tau, g, dataC, g, dataC, x, t);  
  
  % Is the value safe?
  if valuex <= 0, safe = false;
  else            safe = true;
  end
  
  % Compute optimal safe controller
  uSafe = [(gradx(2)>=0)*obj.uMax + (gradx(2)<0)*obj.uMin; ...
    (gradx(4)>=0)*obj.uMax + (gradx(4)<0)*obj.uMin];  
    
else
  error('Safety value function must be 2 or 3D!')
end

% if nargin<4
%   % If the quadrotor is in a platoon AND the other quadrotor is in a
%   % platoon AND (the platoons are the same OR one of the quadrotors is an
%   % EmergLeader), then use internal separation time.
%   % Otherwise, use external separation time.
%   if ~isempty(obj.p) && ~isempty(other.p) && ...
%       (obj.p.ID == other.p.ID || ...
%       strcmp(obj.q,'EmergLeader') || strcmp(other.q, 'EmergLeader'))
%     t = obj.tauInt+0.5;
%   else
%     t = obj.tauExt+0.5;
%   end
% end


end