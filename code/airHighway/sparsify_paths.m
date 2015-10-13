function sparse_pts = sparsify_paths(dense_pts, threshold)
% function sparse_pts = sparsify_paths(dense_pts, threshold)
%
% Groups nearby points of highways together 

if nargin<2
  threshold = 0.05;
end

pts_list = cell2list(dense_pts);

i = 1;
while i < size(pts_list,2)
  % Find nearby points (recursively)
  near_pts = nearby_pts(pts_list(:,i), pts_list, threshold);
  
  % Find center of mass
  com = center_of_mass(near_pts(1:2,:));
  
  % Trim the long list of points
  [~, inds] = intersect(pts_list', near_pts', 'rows', 'stable');
  pts_list(1,inds') = com(1);
  pts_list(2,inds') = com(2);
  
  % Remove duplicates
  [~, inds] = unique(pts_list([1 2 4],:)', 'rows', 'stable');
  pts_list = pts_list(:,inds');
  
  % Increment index
  i = i + 1;
end

sparse_pts = list2cell(pts_list);
end