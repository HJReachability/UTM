function computeRS(obj, type)
% computeRS(obj, type)
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

% Computes base reachable sets
switch type
  case 'qr_abs_target_V'
    %% Create platoon / merge onto highway for quadrotors
    % Compute 2D value function that could be used to reconstruct the 4D
    % value function. This computation assumes that the direction of motion
    % is in the positive x-axis
    target = [0 obj.hw_speed 0 0];
    
    % Big, coarse set
    near = 0;
    [grids, datas, tau] = quad_abs_target_2D(target, near, visualize);
    
    gridLim = ...
      [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
    [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

    obj.qr_atcV.g = TTR_out.g;
    obj.qr_atcV.data = TTR_out.value;
    obj.qr_atcV.grad = TTR_out.grad;
    obj.qr_atcV.tau = tau;
    
    % Small, fine set
    near = 1;
    [grids, datas, tau] = quad_abs_target_2D(target, near, visualize);
    
    gridLim = ...
      [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
    [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

    obj.qr_atfV.g = TTR_out.g;
    obj.qr_atfV.data = TTR_out.value;
    obj.qr_atfV.grad = TTR_out.grad;
    obj.qr_atfV.tau = tau;    
    
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
    
  case 'qr_qr_safe_V'
    %% Safety between two quadrotors
    d = 2; % Collision "radius"
    [data, g, tau] = quad_quad_collision_2D(d, visualize);

    if fourD
    % Reconstruct the base reachable set
      gridLim = [g.min-1 g.max+1; g.min-1 g.max+1];
      [~, ~, TTR_out] = ...
        recon2x2D(tau, {g; g}, {data; data}, gridLim, tau(end));
      
      obj.qr_qr_safe_V.g = TTR_out.g;
      obj.qr_qr_safe_V.data = TTR_out.value;
      obj.qr_qr_safe_V.grad = TTR_out.grad;
      obj.qr_qr_safe_V.tau = tau;      
      
    else 
      obj.qr_qr_safe_V.g = g;
      obj.qr_qr_safe_V.data = data;
      obj.qr_qr_safe_V.tau = tau;
      obj.qr_qr_safe_V.grad = [];      
    end
    
  case 'pl_create_platoon_V'
    error('Not implemented yet.')
    
  case 'pl_join_platoon_V'
    error('Not implemented yet.')
    
  case 'pl_pl_safe_V'
    %% Safety between two Planes
    % 3D sets take a while to compute... load from file if available;
    % otherwise, compute and save for next time.
    filename = ['../RS_core/saved/pl_pl_safe_V_' ...
      num2str(obj.collision_radius) '_' num2str(obj.hw_speed) '.mat'];
    
    if exist(filename, 'file')
      load(filename)
    else
      [g, data, grad] = ...
        pl_pl_collision_3D(obj.collision_radius, obj.hw_speed, ...
        visualize);
      save(filename, 'g', 'data', 'grad')
    end
    
    obj.pl_pl_safe_V.g = g;
    obj.pl_pl_safe_V.data = data;
    obj.pl_pl_safe_V.grad = grad;
    
  otherwise
    error('Undefined reachable set type.')
end
end % end function