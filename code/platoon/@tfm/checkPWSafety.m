function [safe, uSafe] = checkPWSafety(obj, i, j)
% function checkPWSafety(obj, i, j)
%          (check pairwise safety)
%
% Checks the safety of active agent i with respect to active agent j, and
% gives a safety-preserving input if necessary
%
% Inputs:  obj - tfm object
%          i   - index of agent for whom safety is checked
%          j   - index of agent with whom safety is checked against
% Outputs: safe - boolean specifying whether agent i is safe with respect
%                 to agent j
%          uSafe - safety-preserving input (empty if not needed)
%
% Mo Chen, 2015-11-03

% Vehicle is safe with respect to itself
if obj.aas{i} == obj.aas{j}
  safe = 1;
  uSafe = [];
  return;
end

switch(class(obj.aas{i}))
  case 'Quadrotor'
    %% agent i is a quadrotor
    switch(class(obj.aas{j}))
      case 'Quadrotor'
        [safe, uSafe] = checkPWSafety_qr_qr(obj.qr_qr_safeV, ...
          obj.safetyTime, obj.aas{i}, obj.aas{j});
        
      case 'Platoon'
        if ~isa(obj.aas{j}.vehicles{1}, 'Quadrotor')
          error('Unknown platoon type!')
        end
        [safe, uSafe] = checkPWSafety_qr_qrp(obj.aas{i}, obj.aas{j});
        
      otherwise
        error('Unknown agent type')
    end % end inner switch
    
  case 'Platoon'
    %% agent i is a platoon
    if ~isa(obj.aas{i}.vehicles{1}, 'Quadrotor')
      error('Unknown platoon type!')
    end
    
    switch(class(obj.aas{j}))
      case 'Quadrotor'
        [safe, uSafe] = checkPWSafety_qrp_qr(obj.aas{i}, obj.aas{j});
        
      case 'Platoon'
        if ~isa(obj.aas{j}.vehicles{i}, 'Quadrotor')
          error('Unknown platoon type!')
        end
        [safe, uSafe] = checkPWSafety_qrp_qrp(obj.aas{i}, obj.aas{j});
        
      otherwise
        error('Unknown agent type')
    end % end inner switch
    
  otherwise
    error('Unknown agent type')
end % end outer switch

end % end function

%%
function [safe, uSafe] = checkPWSafety_qr_qr(qr_qr_safeV, safetyTime, ...
  qr1, qr2)
% Safety of quadrotor qr1 with respect to quadrotor qr2

% Heading of "pursuer"
theta = qr2.getHeading;

% Get relative state assuming pursuer faces 0 degrees
base_pos = rotate2D(qr2.getPosition - qr1.getPosition, -theta);
base_vel = rotate2D(qr2.getVelocity - qr1.getVelocity, -theta);
base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];

% Check if state is within grid bounds for a closer safety check
if any(base_x <= qr_qr_safeV.g.min) || ...
    any(base_x >= qr_qr_safeV.g.max)
  safe = 1;
  uSafe = [];
  return
end

% Compute safety value
valuex = eval_u(qr_qr_safeV.g, qr_qr_safeV.data, base_x);

% Compute safety preserving control if needed
if valuex > safetyTime
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafe = [];
else
  % Not safe, compute safety controller
  safe = 0;

  % Compute control assuming "pursuer" is facing 0 degrees
  base_grad = calculateCostate(qr_qr_safeV.g, qr_qr_safeV.grad, base_x);
  ux = (base_grad(2)>=0)*qr1.uMin + (base_grad(2)<0)*qr1.uMax;
  uy = (base_grad(4)>=0)*qr1.uMin + (base_grad(4)<0)*qr1.uMax;
  u = [ux; uy];
  
  % Rotate the control to correspond with the actual heading of the
  % "pursuer"
  uSafe = rotate2D(u, theta);  
  
end

end

%%
function [safe, uSafe] = checkPWSafety_qr_qrp(qr, qrp)
% Safety of quadrotor qr with respect to quadrotor platoon qrp
error('Not implemented yet')
end

function [safe, uSafe] = checkPWSafety_qrp_qr(qrp, qr)
% Safety of quadrotor platoon qrp with respect to quadrotor qr
error('Not implemented yet')
end

function [safe, uSafe] = checkPWSafety_qrp_qrp(qrp1, qrp2)
% Safety of quadrotor platoon qrp1 with respect to quadrotor platoon qrp2
error('Not implemented yet')
end