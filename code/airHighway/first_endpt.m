function ind = first_endpt(path, Navg, threshold)
% function ind = first_endpt(path, Navg, threshold)
%
% Given a path, returns the first highway end point, determined by a change
% in direction of more than threshold degrees.
%
% Inputs:  path      - a list of points representing the path
%          Navg      - number of points around any given point on path for
%                      which the average is taken in order to compute the
%                      direction
%          threshold - angle threshold in degrees at which a new segment of
%                      highway begins
%
% Output: ind        - the index of path at which a new highway segment
%                      begins
%
% Mo Chen, 2015-09-30

% Default number of angles to average
if nargin<2
  Navg = 0;
end

% Default angle threshold (in degrees) for ending a highway
if nargin<3
  threshold = 15;
end

% Convert angle to radians
threshold = threshold * pi / 180;

% Initial heading
x0 = path(1,1);
y0 = path(2,1);
theta0 = angle_mean( ...
  atan2(y0-path(2,2:2+2*Navg), x0-path(1,2:2+2*Navg)) );
ind = 2+Navg;

% Subsequent headings along the path
for i = 3+Navg:size(path,2)-Navg
  x0 = path(1,i-Navg-1);
  y0 = path(2,i-Navg-1);
  theta = angle_mean(atan2(y0-path(2,i-Navg:i+Navg), x0-path(1,i-Navg:i+Navg)) );
  
  diff = abs_angle_diff(theta, theta0);
  
  ind = i;
  if diff >= threshold
    break;
  end
end

end