function u = getToRelpos(obj, veh, veh_ref, relpos, debug)

switch class(veh)
  case 'Quadrotor'
    %% ===== Main vehicle is a quadrotor =====
    switch class(veh_ref)
      case 'Quadrotor'
        %% Reference vehicle is a quadrotor
        u = getToRelpos_qr_qr(obj, veh, veh_ref, relpos, debug);
        
      otherwise
        %% Otherwise
        error('This has not been implemented yet.')
    end
    
  otherwise
    %% ===== Otherwise =====
    error('This has not been implemented yet.')
end
end

function u = getToRelpos_qr_qr(obj, veh, veh_ref, relpos, debug)
% u = getToRelpos_qr_qr(obj, veh, veh_ref, relpos, debug)

% Reference vehicle heading
heading = veh_ref.getHeading;

base_pos = ...
  rotate2D(veh.getPosition - veh_ref.getPosition - relpos, -heading);
base_vel = rotate2D(veh.getVelocity - veh_ref.getVelocity, -heading);
base_x = [base_pos(1) base_vel(1) base_pos(2) base_vel(2)];
valuex = eval_u(obj.qr_rel_target_V.g, obj.qr_rel_target_V.data, base_x);

%% At target
if valuex <= obj.ttt
  if debug
    disp('Arrived')
  end
    u = [];
    return;
end

%% In reachable set
if valuex <= obj.rtt
  disp(['In reachable set; valuex = ' num2str(valuex)])
  % Gradient in the frame of base reachable set
  base_p = calculateCostate(obj.qr_rel_target_V.g, ...
                                        obj.qr_rel_target_V.grad, base_x);
  
  ux = (base_p(2)>=0)*veh.uMin + (base_p(2)<0)*veh.uMax;
  uy = (base_p(4)>=0)*veh.uMin + (base_p(4)<0)*veh.uMax;
  u = rotate2D([ux; uy], heading);
  return;
end

%% Outside of reachable set
if debug
  disp('Outside reachable set')
end

% Amount behind the target set to aim for (based on target heading)
behind_amount = 0;
behind_target = ...
  veh_ref.getPosition - behind_amount*[cos(heading); sin(heading)];

waypoint = ...
  Linpath(veh.getPosition, behind_target, 1.2*obj.hw_speed);
u = veh.followPath(waypoint);
end