function plot_abs_target_V(obj, abs_target_V, target_position, ...
  target_heading, level)
% plot_abs_target_V(obj, abs_target_V, target_position, ...
%   target_heading, level)
%
% Plots the reachable set for creating a platoon

if nargin < 4
  level = 0:10;
end

if numel(level) == 1
  level = [level level];
end

% Project reachable set to 2D
[g2D, data2D] = proj2D(abs_target_V.g, abs_target_V.data, [0 1 0 1], ...
  rotate2D(obj.getVelocity, -target_heading));

% translation and rotation
gRot = rotateGrid(g2D, target_heading);
gFinal = shiftGrid(gRot, target_position);

% Plot result
if isempty(obj.h_abs_target_V)
  [~, obj.h_abs_target_V] = contour(gFinal.xs{1}, gFinal.xs{2}, data2D, ...
    level, 'linestyle', ':', 'linewidth', 2);
else
  obj.h_abs_target_V.XData = gFinal.xs{1};
  obj.h_abs_target_V.YData = gFinal.xs{2};
  obj.h_abs_target_V.ZData = data2D;
  obj.h_abs_target_V.Visible = 'on';
end
end