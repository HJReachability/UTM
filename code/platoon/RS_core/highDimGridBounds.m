function [xmin, xmax] = highDimGridBounds(g, reference)
% function [xmin, xmax] = highDimGridBounds(g, reference)
%
% Inputs: g         - grid structure
%         reference - a vector of twice the length of g.dim, in which NaN
%                     elements represent dimensions with a large desired
%                     interval, and other elements represent the center of
%                     the neighborhood in those dimensions
%
% Computes the high dimensional grid bounds for reachable set visualization
%
% Mo Chen, 2015-10-20

%% Input checks
if sum(isnan(reference)) ~= 2
  error('There can only be 2 dimensions without reference!')
end

if g.dim ~= 2 && g.dim ~= 3
  error('Grid must be 2D or 3D!')
end

if length(reference) ~= 2 * g.dim
  error('Length of reference must be twice the dimension of grid!')
end

%% Default max_size and thickness and initialization
max_size = 1;
thickness = 2;

xmin = zeros(length(reference),1);
xmax = zeros(length(reference),1);

%% Compute high dimensional grid bounds
for i = 1:length(reference)
  if i <= g.dim
    if isnan(reference(i))
      % For dimensions without reference, use a large interval
      xmin(i) = max_size * g.min(i);
      xmax(i) = max_size * g.max(i);
    else
      % For dimensions with a reference, use a neighborhood around the
      % reference
      xmin(i) = reference(i) - thickness * g.dx(i);
      xmax(i) = reference(i) + thickness * g.dx(i);
    end
  else
    % Same thing for the second replication of dimensions, but now we need
    % to subtract grid indices
    if isnan(reference(i))
      xmin(i) = max_size * g.min(i - g.dim);
      xmax(i) = max_size * g.max(i - g.dim);
    else
      xmin(i) = reference(i) - thickness * g.dx(i - g.dim);
      xmax(i) = reference(i) + thickness * g.dx(i - g.dim);
    end
  end
end

end