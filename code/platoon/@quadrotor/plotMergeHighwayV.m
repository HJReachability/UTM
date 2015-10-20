function plotMergeHighwayV(obj, target, hw)
% function plotMergeHighwayV(obj, target)
%
% Plots the zero sublevel set of value function for merging onto the
% highway
%
% Inputs:  obj    - quadrotor object
%          target - target state that quadrotor is aiming to reach
%
% Mo Chen, 2015-06-21

% Unpack constants

%% Construct grid
% Position domain should cover all grid positions around the OTHER vehicle
% since p = [px py] indicates that this vehicle is at (px, py) where the
% origin is centered around the other vehicle
%
% Velocity domain should cover a thin layer around current relative
% velocity
if hw.liveV.g.dim == 2
  reference = zeros(obj.nx, 1);
  reference(obj.pdim) = nan;
  reference(obj.vdim) = obj.getVelocity;
  [xmin, xmax] = highDimGridBounds(hw.liveV.g, reference);
  
  % Compute value for V(t,x) on the relative velocity slice and project down
  % to 2D
  [g2D, value2D] = reconProj2D(hw.liveV, xmin, xmax, inf, obj.getVelocity);
  
  % Shift the grid!!!
  g2Dt.dim = g2D.dim;
  g2Dt.min = g2D.min + target';
  g2Dt.max = g2D.max + target';
  g2Dt.N = g2D.N;
  g2Dt.bdry = g2D.bdry;
  g2Dt = processGrid(g2Dt);
end

% Plot result
if isempty(obj.hmergeHighwayV)
  [~, obj.hmergeHighwayV] = contour(g2Dt.xs{1}, g2Dt.xs{2}, value2D, [0 0], ...
    'lineStyle', ':', 'linewidth', 2);
else
  obj.hmergeHighwayV.XData = g2Dt.xs{1};
  obj.hmergeHighwayV.YData = g2Dt.xs{2};
  obj.hmergeHighwayV.ZData = value2D;
  obj.hmergeHighwayV.Visible = 'on';
end

% Color
if isempty(obj.hpxpyhist.Color)
  obj.hmergeHighwayV.LineColor = 'r';
else
  obj.hmergeHighwayV.LineColor = obj.hpxpyhist.Color;
end

end