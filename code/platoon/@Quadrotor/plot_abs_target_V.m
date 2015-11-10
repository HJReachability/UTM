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

%% Coarse
% Project reachable set to 2D
[g2D, data2D] = proj2D(abs_target_V.g, [0 1 0 1], ...
  abs_target_V.g.N(obj.pdim), abs_target_V.value, ...
  rotate2D(obj.getVelocity, -theta));

% translation and rotation
gRot = rotateGrid(g2D, theta);
gFinal = shiftGrid(gRot, shift);

% Plot result
if isempty(obj.h_abs_target_V)
  [~, obj.h_abs_target_V] = contour(gFinal.xs{1}, gFinal.xs{2}, data2D, ...
    0:10, 'linestyle', ':', 'linewidth', 2);
else
  obj.h_abs_target_V.XData = gFinal.xs{1};
  obj.h_abs_target_V.YData = gFinal.xs{2};
  obj.h_abs_target_V.ZData = data2D;
  obj.h_abs_target_V.Visible = 'on';
end
end