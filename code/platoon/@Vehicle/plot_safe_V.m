function plot_safe_V(obj, other, safe_V, level)
% plot_safe_V(obj, other, safe_V, level)
%
% Plots the safety reachable set of this vehicle with respect to the other
% vehicle
%
% Inputs: obj, other - this and other vehicle
%         safe_V     - safety value function
%         level      - the level of the safety value function to visualize
%
% Mo Chen, 2015-11-14
% Modified: 2016-03-03, Mo Chen

if nargin < 4
  level = 0:0.5:5;
end

if numel(level) == 1
  level = [level level];
end

shift = other.getPosition;
theta = obj.getHeading;

% Project reachable set to 2D
switch safe_V.g.dim
  case 3
    rel_heading = other.getHeading - obj.getHeading;
    while rel_heading >= 2*pi
      rel_heading = rel_heading - 2*pi;
    end

    while rel_heading < 0
      rel_heading = rel_heading + 2*pi;
    end
    
    [g2D, data2D] = proj2D(safe_V.g, safe_V.data, [0 0 1], rel_heading);
    
    gRot = rotateGrid(g2D, other.getHeading);
  case 4
    [g2D, data2D] = proj2D(safe_V.g, safe_V.data, [0 1 0 1], ...
      rotate2D(obj.getVelocity - other.getVelocity, -theta));
    
    % rotation
    gRot = rotateGrid(g2D, theta);
    
  otherwise
    error([mfilename ' has not been implemented for ' num2str(g.dim) ...
      ' dimensional systems!'])
end

% Translation
gFinal = shiftGrid(gRot, shift);

% Plot result
if isempty(obj.h_safe_V)
  [~, obj.h_safe_V] = contour(gFinal.xs{1}, gFinal.xs{2}, data2D, ...
    level, 'linestyle', '--', 'linewidth', 2, 'color', obj.hpxpy.Color);
else
  obj.h_safe_V.XData = gFinal.xs{1};
  obj.h_safe_V.YData = gFinal.xs{2};
  obj.h_safe_V.ZData = data2D;
  obj.h_safe_V.Visible = 'on';
end

end