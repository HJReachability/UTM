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
% Updated by Mahesh Vashishtha, 2015-12-03

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
    
    filename = [fileparts(mfilename('fullpath')) ...
      '/../RS_core/saved/qr_abs_target_V_' ...
      num2str(obj.hw_speed) '.mat'];
    
    if exist(filename, 'file')
      load(filename)
    else
      [grids, datas, tau] = quad_abs_target_2D(target);

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
    
    filename = [fileparts(mfilename('fullpath')) ...
      '/../RS_core/saved/qr_rel_target_V.mat'];
    
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
    filename = [fileparts(mfilename('fullpath')) ... 
      '/../RS_core/saved/qr_qr_safe_V_' ...
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
  
   case 'qr_pl4_safe_V'
    %% Safety for quadrotor pursuing a plane4
    filename = [fileparts(mfilename('fullpath')) ... 
      '/../RS_core/saved/qr_pl4_safe_V_' ...
      num2str(obj.cr) '_' num2str(obj.hw_speed) '.mat'];

    if exist(filename, 'file')
      load(filename)
    else
      [data, g, tau] = ...
        quad_pl4_collision_2D(obj.cr, obj.hw_speed, visualize);

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
    obj.qr_pl4_safe_V.g = g;
    obj.qr_pl4_safe_V.data = data;
    obj.qr_pl4_safe_V.grad = grad;
    obj.qr_pl4_safe_V.tau = tau; 

   case 'pl4_qr_safe_V'
    %% Safety for plane4 pursuing a quadrotor
    filename = [fileparts(mfilename('fullpath')) ... 
      '/../RS_core/saved/pl4_qr_safe_V_' ...
      num2str(obj.cr) '_' num2str(obj.hw_speed) '.mat'];

    if exist(filename, 'file')
      load(filename)
    else
      [data, g, tau] = ...
        pl4_quad_collision_2D(obj.cr, obj.hw_speed, visualize);

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
    obj.pl4_qr_safe_V.g = g;
    obj.pl4_qr_safe_V.data = data;
    obj.pl4_qr_safe_V.grad = grad;
    obj.pl4_qr_safe_V.tau = tau; 
    
  case 'pl_pl_safe_V'
    %% Safety between two Planes
    % 3D sets take a while to compute... load from file if available;
    % otherwise, compute and save for next time.
    filename = [fileparts(mfilename('fullpath')) ...
      '/../RS_core/saved/pl_pl_safe_V_' ...
      num2str(obj.cr) '_' num2str(obj.hw_speed) '.mat'];
    
    if exist(filename, 'file')
      load(filename)
    else
      [g, data, grad] = ...
        pl_pl_collision_3D(obj.cr, obj.hw_speed, ...
        visualize);
      save(filename, 'g', 'data', 'grad')
    end
    
    obj.pl_pl_safe_V.g = g;
    obj.pl_pl_safe_V.data = data;
    obj.pl_pl_safe_V.grad = grad;
    
  case 'pl4_pl4_safe_V'
    %% Safety between two Plane4's
    filename = [fileparts(mfilename('fullpath')) ... 
      '/../RS_core/saved/pl4_pl4_safe_V_' ...
      num2str(obj.cr) '_' num2str(obj.hw_speed) '.mat'];

    if exist(filename, 'file')
      load(filename)
    else
      [data, g, tau] = ...
        pl4_pl4_collision_2D(obj.cr, obj.hw_speed, visualize);
      
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
        error('Plane4 must be 4D');
      end
    end
    obj.pl4_pl4_safe_V.g = g;
    obj.pl4_pl4_safe_V.data = data;
    obj.pl4_pl4_safe_V.grad = grad;
    obj.pl4_pl4_safe_V.tau = tau;
    
  case 'pl4_rel_target_V'
    %% Join platoon / merge onto highway for plane4
    x = [0 0 0 0]; % Base reachable set assumes 0 relative state
    
    filename = [fileparts(mfilename('fullpath')) ...
      '/../RS_core/saved/pl4_rel_target_V.mat'];
    
    if exist(filename, 'file')
      load(filename)
    else
      [g, data, TTR] = pl4_rel_target_4D('medium');
      grad = extractCostates(g, data);    
      save(filename, 'g', 'data', 'grad', 'TTR')
    end
    
    obj.pl4_rel_target_V.g = g;
    obj.pl4_rel_target_V.data = data;
    obj.pl4_rel_target_V.grad = grad;
    %obj.pl4_rel_target_V.tau = tau;
    
  otherwise
    error('Undefined reachable set type.')
end
end % end function