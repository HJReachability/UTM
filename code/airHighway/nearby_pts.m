function near_pts = nearby_pts(this_pt, other_pts, threshold)
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

% Compute distance to this_pt from other_pts
dist = (this_pt(1) - other_pts(1,:)).^2 + (this_pt(2) - other_pts(2,:)).^2;
dist = sqrt(dist);

% Find nearby points
near_pts = other_pts(:, dist <= threshold);

end