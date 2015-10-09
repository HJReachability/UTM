function chosen_pts = nearby_pts(this_pt, other_pts, threshold)
% function near_pts = nearby_pts(this_pt, other_pts, threshold)
%
% Finds points close to this_pt within a distance specified by threshold in
% the set of other_pts
%
% Inputs: this_pt - reference point
%         other_pts - set of points to search in
%         threshold - find all points in other_pts that are a distance of
%                     less than or equal to threshold to this_pt
%
% Output: near_pts - resulting points
%
% Mo Chen, 2015-10-08

if threshold <= 0
  error('threshold must be strictly positive!')
end

% Append indices to set of search points if needed
if size(other_pts,1) == 2
  other_pts = [other_pts; 1:size(other_pts,2)];
elseif size(other_pts,1) < 2
  error('Input set of points must contain at least 2 rows!')
end

% Compute distance to this_pt from other_pts
dist = (this_pt(1) - other_pts(1,:)).^2 + (this_pt(2) - other_pts(2,:)).^2;
dist = sqrt(dist);

% Find nearby points
logical_inds = dist<=threshold;
chosen_pts = other_pts(:, logical_inds);
remaining_pts = other_pts(:, ~logical_inds);

for i = 1:size(chosen_pts,2)
  chosen_pts = [chosen_pts nearby_pts(chosen_pts(:,i), remaining_pts, ...
    threshold)];
end

end