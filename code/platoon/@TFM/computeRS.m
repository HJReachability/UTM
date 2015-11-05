function computeRS(obj, type)
% function computeRS(obj, type)
%
% Computes a reachable set specified by its type. Currently, the available
% types are as follows:
%
%   qr_create_platoon_V
%   qr_join_platoon_V
%   qr_qr_safeV
%
% Mo Chen, 2015-11-03

% For now, always assume we have a reconstructed set; deal with
% reconstruction on the fly later...
fourD = true;
visualize = false;
speed = obj.hw_speed;
% Computes base reachable sets
switch type
  case 'qr_create_platoon_V'
    %% Create platoon / merge onto highway for quadrotors
    % Compute 2D value function that could be used to reconstruct the 4D
    % value function. This computation assumes that the direction of motion
    % is in the positive x-axis
    target = [0 speed 0 0];
    [grids, datas, tau] = quad_abs_target_2D(target, visualize);
    
    % If 4D reachable set is specified, then reconstruct it; otherwise,
    % simply store the 2D reachable sets and reconstruct later
    
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
    x = [0 0 0 0]; % Base reachable set assumes 0 relative state
    [grids, datas, tau] = quad_rel_target_2D(x, visualize);
    
    % If 4D reachable set is specified, then reconstruct it; otherwise,
    % simply store the 2D reachable sets and reconstruct later
    if fourD
      gridLim = ...
        [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
      [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));
      
      obj.qr_join_platoon_V.g = TTR_out.g;
      obj.qr_join_platoon_V.data = TTR_out.value;
      obj.qr_join_platoon_V.grad = TTR_out.grad;
      obj.qr_join_platoon_V.tau = tau;
      
    else
      obj.qr_join_platoon_V.g = grids;
      obj.qr_join_platoon_V.data = datas;
      obj.qr_join_platoon_V.tau = tau;
      obj.qr_join_platoon_V.grad = [];
    end
    
  case 'qr_qr_safeV'
    %% Safety between two quadrotors
    d = 2; % Collision "radius"
    [data, g, tau] = quad_quad_collision_2D(d, visualize);

    if fourD
    % Reconstruct the base reachable set
      gridLim = [g.min-1 g.max+1; g.min-1 g.max+1];
      [~, ~, TTR_out] = ...
        recon2x2D(tau, {g; g}, {data; data}, gridLim, tau(end));
      
      obj.qr_qr_safeV.g = TTR_out.g;
      obj.qr_qr_safeV.data = TTR_out.value;
      obj.qr_qr_safeV.grad = TTR_out.grad;
      obj.qr_qr_safeV.tau = tau;      
      
    else 
      obj.qr_qr_safeV.g = g;
      obj.qr_qr_safeV.data = data;
      obj.qr_qr_safeV.tau = tau;
      obj.qr_qr_safeV.grad = [];      
    end
    
  case 'pl_create_platoon_V'
  case 'pl_join_platoon_V'
  case 'pl_pl_safe_V'
  otherwise
    error('Undefined reachable set type.')
end
end % end function