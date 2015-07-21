function popuHwyConnections(hw)
% function populateConnections(highways)
% Computes part of the connections property for an array of highways
%
% Input: highways - a cell srtucture of highway objects
%
% Mo Chen, 2015-05-27
% Modified, 2015-07-21

% Initialize the connections property
num_hw = length(hw);
for i = 1:num_hw
  hw{i}.connections = zeros(num_hw,1);
end

% Go through each combination and compute intersections
for i = 1:num_hw
  for j = i+1:num_hw
    [si, sj] = hwInt(hw{i}, hw{j});
    
    hw{i}.connections(j) = si;
    hw{j}.connections(i) = sj;
    
  end
end

end