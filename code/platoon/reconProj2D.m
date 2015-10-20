function [g2D, value2D] = reconProj2D(V, xmin, xmax, t, slice)
% function [g2D, value2D] = create2DSafeVData(safeV, xmin, xmax, t)
%
% Given a low dimensional reachable set, reconstruct a higher dimensional
% reachable set in a slice of the higher dimensional space, and then
% project the result back down to 2D for plotting
%
% Inputs:  safeV      - raw safety reachable set to be reconstructed
%          xmin, xmax - high dimensional grid bounds for reconstruction
%          t          - time horizon for reconstruction
% Outputs: g2D        - 2D grid structure for plotting (to be shifted)
%          value2D    - 2D array for plotting
%
% Mo Chen, 2015-10-20

% Grid check
if g.dim ~= 2 && g.dim ~= 3
  error('Reconstruction requires the grid to be 2D or 3D!')
end

% Unpack constants
tau = V.tau;
g = V.g;

if g.dim == 3
  dataC = V.dataC;
  dataS = V.dataS;
else
  dataC = V.dataC;
end

if g.dim == 3
  % Collision reachable set: reconstruct the "max" problem
  [~, ~, g6D, valueC, ~, ind] = ...
    recon2x3D(tau, g, dataC, g, dataC, [xmin xmax], t);
  
  % Velocity reachable set: take union for the "min" problem
  valueSx = eval_u(g, dataS(:,:,:,ind), 
    [g6D.xs{1}(:) g6D.xs{2}(:) g6D.xs{3}(:)]);
  valueSy = eval_u(g, dataS(:,:,:,ind), ...
    [g6D.xs{4}(:) g6D.xs{5}(:) g6D.xs{6}(:)]);
  valueS = min(valueSx, valueSy);
  valueS = reshape(valueS, g6D.shape);
  
  % Overall safety set is the union of collision and velocity limit sets
  value = min(valueC, valueS);
  
  % Project to 2D
  [g2D, value2D] = proj2D(g6D, [0 1 1 0 1 1], g6D.N([1 4]), value, slice);
  
else
  % Collision reachable set: reconstruct the "max" problem
  [~, TD_out] = recon2x2D(tau, {g; g}, {dataC; dataC}, [xmin xmax], t);
  
  % Overall safety set is the collision safety set
  g4D = TD_out.g;
  value = TD_out.value;
  
  % Project to 2D
  [g2D, value2D] = proj2D(g4D, [0 1 0 1], g4D.N([1 3]), value, slice);
end
end