clear all
% Specify some arbitrary points in a cell structure
points_in_cell = {[0.1 0.2 0.3 0.4; rand(1,4)]; ...
  [1 2 3 4 5; rand(1,5)]; ...
  [100 200 300; 10*rand(1,3)]};
points_in_cell{:}

% Convert to list
long_list_of_points = cell2list(points_in_cell)

% Convert back to cell
original_cell = list2cell(long_list_of_points);
original_cell{:}