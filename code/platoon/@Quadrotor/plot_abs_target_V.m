function plot_abs_target_V(obj, atcV, atfV, clevel, flevel, shift, theta)
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
[g2D, data2D] = proj2D(atcV.g, [0 1 0 1], atcV.g.N(obj.pdim), ...
  atcV.data, rotate2D(obj.getVelocity, -theta));

% translation and rotation
gRot = rotate2DGrid(g2D, theta);
gFinal = shift2DGrid(gRot, shift);

% Plot result
if isempty(obj.h_atcV)
  [~, obj.h_atcV] = contour(gFinal.xs{1}, gFinal.xs{2}, data2D, ...
    [clevel clevel], 'linestyle', ':', 'linewidth', 2);
else
  obj.h_atcV.XData = gFinal.xs{1};
  obj.h_atcV.YData = gFinal.xs{2};
  obj.h_atcV.ZData = data2D;
  obj.h_atcV.Visible = 'on';
end

%% Fine
% Project reachable set to 2D
[g2D, data2D] = proj2D(atfV.g, [0 1 0 1], atfV.g.N(obj.pdim), ...
  atfV.data, rotate2D(obj.getVelocity, -theta));

% translation and rotation
gRot = rotate2DGrid(g2D, theta);
gFinal = shift2DGrid(gRot, shift);

% Plot result
if isempty(obj.h_atfV)
  [~, obj.h_atfV] = contour(gFinal.xs{1}, gFinal.xs{2}, data2D, ...
    [flevel flevel], 'linestyle', ':', 'linewidth', 2);
else
  obj.h_atfV.XData = gFinal.xs{1};
  obj.h_atfV.YData = gFinal.xs{2};
  obj.h_atfV.ZData = data2D;
  obj.h_atfV.Visible = 'on';
end
end