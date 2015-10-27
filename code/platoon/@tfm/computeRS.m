function computeRS(obj, type)
% Computes base reachable sets
switch type
  case 'qr_create_platoon_V'
    %% Create platoon / merge onto highway for quadrotors
    % Compute 2D value function that could be used to reconstruct the 4D
    % value function. This computation assumes that the direction of motion
    % is in the positive x-axis
    visualize = false;
    target = [0 speed 0 0];
    [grids, datas, tau] = quad_abs_target_2D(target, visualize);
    
    % If 4D reachable set is specified, then reconstruct it; otherwise,
    % simply store the 2D reachable sets and reconstruct later
    fourD = true;
    if fourD
      gridLim = ...
        [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
      [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));
      
      obj.qr_create_platoon_V.g = TTR_out.g;
      obj.qr_create_platoon_V.data = TTR_out.value;
      obj.qr_create_platoon_V.grad = TTR_out.grad;
      obj.qr_create_platoon_V.tau = tau;
      
    else
      obj.qr_create_platoon_V.g = grids;
      obj.qr_create_platoon_V.data = datas;
      obj.qr_create_platoon_V.tau = tau;
      obj.qr_create_platoon_V.grad = [];
    end
    
  case 'qr_join_platoon_V'
    %% Join platoon / merge onto highway for quadrotors
  case 'qr_qr_SafeV'
  case 'pl_create_platoon_V'
  case 'pl_join_platoon_V'
  case 'pl_pl_safe_V'
  otherwise
    error('Undefined reachable set type.')
end
end % end function