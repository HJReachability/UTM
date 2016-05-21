function pts = path2hws(path, Navg, threshold)
% function pts = path2hws(path)
%
% Converts a finely spaced sequence of points (eg. from FMM) to a sparse
% sequence of waypoints which form the start and ending points of air
% highways
%
% Input: path - 2 x N matrix representing the sequence of points
%        threshold - angle threshold in degrees for starting a new segment
%                    of air highway
% Ouptut: pts - The starting and ending points of air highways
%
% Mo Chen, 2015-09-30

% Default parameters
if nargin<2
  Navg = 0;
end

if nargin<3
  threshold = 15;
end

% Initial point
ind = 1;
pts = [path(1,ind); path(2,ind)];

% Create segments
while size(path,2) > 2*Navg
  ind = first_endpt(path, Navg, threshold);
  pts = [pts [path(1,ind); path(2,ind)]];
  path(:, 1:ind) = [];
end

% Final point
if ~isempty(path)
  pts = [pts [path(1,end); path(2,end)]];
end
end