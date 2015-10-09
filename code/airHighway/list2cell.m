function points_in_cell = list2cell(long_list_of_points)
% function points_in_cell = list2cell(long_list_of_points)
%
% Inverse function of long_list_of_points
%
% Mo Chen, 2015-10-09

% Initialize cell structure
cell_size = max(long_list_of_points(4,:));
points_in_cell = cell(cell_size, 1);
for i = 1:cell_size
  points_in_cell{i} = [];
end

% Convert to cell structure
for i = 1:size(long_list_of_points,2)
  j = long_list_of_points(4,i);
  points_in_cell{j} = [points_in_cell{j} long_list_of_points(1:2,i)];
end
end