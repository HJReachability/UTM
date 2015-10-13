function pts = nearby_pts(pts, other_pts, threshold)
% function near_pts = nearby_pts(pts, other_pts, threshold)
%
% Finds points close to this_pt within a distance specified by threshold in
% the set of other_pts
%
% Inputs: pts       - reference points
%         other_pts - set of points to search in
%         threshold - find all points in other_pts that are a distance of
%                     less than or equal to threshold to any point in pts
%
% Output: pts - resulting points
%
% Mo Chen, 2015-10-08

if threshold <= 0
  error('threshold must be strictly positive!')
end

% Append indices to set of search points if needed
if size(pts,1) == 2
  pts = [pts; zeros(1, size(pts,2))];
elseif size(pts,1) < 2
  error('Input set of points must contain at least 2 rows!')
end

% Append indices to set of search points if needed
if size(other_pts,1) == 2
  other_pts = [other_pts; 1:size(other_pts,2)];
elseif size(other_pts,1) < 2
  error('Input set of points must contain at least 2 rows!')
end

% Go through the list of accepted points, starting with the input points
i = 1;
while i <= size(pts, 2)
  % Check distance to the set of other points
  dist = (pts(1,i) - other_pts(1,:)).^2 + (pts(2,i) - other_pts(2,:)).^2;
  dist = sqrt(dist);
  
  % Pick out those points with distance less than the threshold
  logical_inds = dist<=threshold;
  
  % Add those points to accepted, and remove them from the set of other
  % points
  if ~isempty(logical_inds)
    pts = [pts other_pts(:, logical_inds)];
    other_pts(:, logical_inds) = [];
  end
  
  % Keep iterating
  i = i + 1;
end

end