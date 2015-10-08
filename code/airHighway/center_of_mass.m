function com = center_of_mass(points, weights)
% function com = center_of_mass(points, weights)
% 
% Computes the center of mass of a collection of points
%
% Inputs: points  - 2 by N matrix; each column is a point in 2D space
%         weights - 1 by N matrix specifying the weight of each point
% Output: com     - center of mass; 2 by 1 vector
%
% Mo Chen, 2015-10-08

if size(points, 1) ~= 2
  error('This function currently only works for points in 2D!')
end

N = size(points, 2);

if N <= 2
  warning('There are only at most two points!')
end

if N <= 0
  error('There must be a positive number of points!')
end

% Default to uniform weights
if nargin<2
  weights = ones(1, N);
end

% Compute center of mass

com_x = sum(points(1,:) .* weights)/N;
com_y = sum(points(2,:) .* weights)/N;
com = [com_x; com_y];

end