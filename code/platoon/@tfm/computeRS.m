function computeRS(obj, type)
% Computes base reachable sets
switch type
  case 'qr_create_platoon_V'
    %% Create platoon / merge onto highway for quadrotors
    % Compute 2D value function that could be used to reconstruct the 4D
    % value function
    [grids, datas, tau] = quad2D_liveness( ...
      [0 speed 0 0], 0);
    
    % If 4D reachable set is specified, then reconstruct it; otherwise,
    % simply store the 2D reachable sets and reconstruct later
    if fourD
      gridLim = ...
        [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
      [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));
      
      obj.liveV.g = TTR_out.g;
      obj.liveV.data = TTR_out.value;
      obj.liveV.grad = TTR_out.grad;
      obj.liveV.tau = tau;
      
    else
      obj.liveV.g = grids;
      obj.liveV.data = datas;
      obj.liveV.tau = tau;
      obj.liveV.grad = [];
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