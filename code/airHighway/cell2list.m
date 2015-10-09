function long_list_of_points = cell2list(points_in_cell)
% function long_list_of_points = append_indices(points_in_cell)
% 
% Appends indices to points in a cell structure by adding two rows. The
% first added row specifies the index of the point within the cell element,
% and the second added row specifies the cell index.
%
% Input:  points_in_cell - a cell structure whose elements are 2D points;
%                          each column of each cell element should be a 
%                          point
% Output: long_list_of_points - resulting list of points with appended
%                               indices
%
% Mo Chen, 2015-10-09

if ~iscell(points_in_cell)
  error('Input must be a cell structure!')
end

long_list_of_points = [];

for i = 1:length(points_in_cell)
  if size(points_in_cell{i},1) ~= 2
    error('Points contained in the cell structure must be 2D!')
  end
  
  num_pts = size(points_in_cell{i},2);
  
  % Append index within the cell and cell index  
  augmented_points = [points_in_cell{i}; 1:num_pts; i*ones(1,num_pts)];

  % Add to long list of points
  long_list_of_points = [long_list_of_points augmented_points];
end

end