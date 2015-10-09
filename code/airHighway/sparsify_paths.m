function sparse_pts = sparsify_paths(dense_pts, threshold)
% function sparse_pts = sparsify_paths(dense_pts, threshold)
%
% Groups nearby points of highways together 

if nargin<2
  threshold = 0.25;
end

N = length(dense_pts);

for i = 1:N % Go through all the set of points
  this_pts = dense_pts{i}; % Current point
  
  near_pts = cell(N-1, 1);
  near_inds = cell(N-1, 1);
  
  for j = 1:size(this_pts,2) % Go through all points in the current set
    % Find all nearby points from all sets of points
    for k = [1:i-1 i+1:N]
      [near_pts{k}, near_inds{k}] = ...
        nearby_pts(this_pts(:,j), dense_pts{k}, threshold);
    end
    
    if ~isempty([near_pts{:}]) % If a set of nearby point is found
      % Find center of mass
      com = center_of_mass([near_pts{:}]);
      
      % Replace reference point with center of mass
      dense_pts{i}(:,j) = com;
      
      % Replace all nearby points with center of mass
      for k = [1:i-1 i+1:N] % Go through all other sets of points
        % If there are nearby points, replace with center of mass
        if ~isempty(near_inds{k})
          dense_pts{k}(:,near_inds{k}(1)) = com;
          
          if size(dense_pts{k},2) > 1
            dense_pts{k}(:,near_inds{k}(2:end)) = [];
          end
        end
      end
      
    end
  end
end

sparse_pts = dense_pts;
end