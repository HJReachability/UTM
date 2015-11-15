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
    
    filename = ['../RS_core/saved/qr_abs_target_V_' ...
      num2str(obj.hw_speed) '.mat'];
    if exist(filename, 'file')
      load(filename)
    else
      [grids, datas, tau] = quad_abs_target_2D(target, visualize);

      gridLim = ...
        [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
      [~, ~, TTR_out] = ...
        recon2x2D(tau, grids, datas, gridLim, tau(end));

      g = TTR_out.g;
      data = TTR_out.value;
      grad = TTR_out.grad;
      
      save(filename, 'g', 'data', 'grad', 'tau')
    end
    
    obj.qr_abs_target_V.g = g;
    obj.qr_abs_target_V.data = data;
    obj.qr_abs_target_V.grad = grad;
    obj.qr_abs_target_V.tau = tau;
    
  case 'qr_rel_target_V'
    %% Join platoon / merge onto highway for quadrotors
    x = [0 0 0 0]; % Base reachable set assumes 0 relative state
    
    filename = ['../RS_core/saved/qr_rel_target_V.mat'];
    
    if exist(filename, 'file')
      load(filename)
    else
      [grids, datas, tau] = quad_rel_target_2D(x, visualize);
      
      % If 4D reachable set is specified, then reconstruct it; otherwise,
      % simply store the 2D reachable sets and reconstruct later
      if fourD
        gridLim = ...
          [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
        [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));
        
        g = TTR_out.g;
        data = TTR_out.value;
        grad = TTR_out.grad;
        
      else
        g = grids;
        data = datas;
        tau = tau;
        grad = [];
        
      end
      
      save(filename, 'g', 'data', 'grad', 'tau')
    end
    
    obj.qr_rel_target_V.g = g;
    obj.qr_rel_target_V.data = data;
    obj.qr_rel_target_V.grad = grad;
    obj.qr_rel_target_V.tau = tau;
    
  case 'qr_qr_safe_V'
    %% Safety between two quadrotors
    filename = ['../RS_core/saved/qr_qr_safe_V_' ...
      num2str(obj.cr) '_' num2str(obj.hw_speed) '.mat'];

    if exist(filename, 'file')
      load(filename)
    else
      [data, g, tau] = ...
        quad_quad_collision_2D(obj.cr, obj.hw_speed, visualize);

      if fourD
      % Reconstruct the base reachable set
        gridLim = [g.min-1 g.max+1; g.min-1 g.max+1];
        [~, ~, TTR_out] = ...
          recon2x2D(tau, {g; g}, {data; data}, gridLim, tau(end));
        g = TTR_out.g;
        data = TTR_out.value;
        grad = TTR_out.grad;
        save(filename, 'g', 'data', 'grad', 'tau')
      else
        grad = [];      
      end
    end
    obj.qr_qr_safe_V.g = g;
    obj.qr_qr_safe_V.data = data;
    obj.qr_qr_safe_V.grad = grad;
    obj.qr_qr_safe_V.tau = tau;
    
  case 'pl_create_platoon_V'
    error('Not implemented yet.')
    
  case 'pl_join_platoon_V'
    error('Not implemented yet.')
    
  case 'pl_pl_safe_V'
    %% Safety between two Planes
    % 3D sets take a while to compute... load from file if available;
    % otherwise, compute and save for next time.
    filename = ['../RS_core/saved/pl_pl_safe_V_' ...
      num2str(obj.cr) '_' num2str(obj.hw_speed) '.mat'];
    
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