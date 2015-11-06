function plot_abs_target_V(obj, abs_target_V, level, shift, theta)
% plot_abs_target_V(obj, target, abs_target_V)
%
% Plots the reachable set for creating a platoon
if nargin < 4
  shift = [0 0];
end

if nargin < 5
  theta = 0;
end

% Project reachable set to 2D
[g2D, data2D] = proj2D(abs_target_V.g, [0 1 0 1], ...
  abs_target_V.g.N(obj.pdim), abs_target_V.data, ...
  rotate2D(obj.getVelocity, -theta));

% translation and rotation
gRot = rotate2DGrid(g2D, theta);
gFinal = shift2DGrid(gRot, shift);

% Plot result
if isempty(obj.h_abs_target_V)
  [~, obj.h_abs_target_V] = contour(gFinal.xs{1}, gFinal.xs{2}, data2D, ...
    [level level], 'linestyle', ':', 'linewidth', 2);
else
  obj.hmergeHighwayV.XData = gFinal.xs{1};
  obj.hmergeHighwayV.YData = gFinal.xs{2};
  obj.hmergeHighwayV.ZData = data2D;
  obj.hmergeHighwayV.Visible = 'on';
end
end