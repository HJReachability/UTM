function [safety_value_matrix xr_cell] = compute_pairwise_safety_vals(...
    list_planes, safety_threshold, g, data)
%% Description
% Computes the safety value for each pair of planes
% 
%% Main function body
    num_planes = length(list_planes);
    safety_value_matrix = NaN(num_planes);
    xr_cell = cell(num_planes, num_planes);
    for i=1:num_planes
        for j=1:num_planes
            if i ~= j
                evader = list_planes(i);
                pursuer = list_planes(j);
                [safety_value xr] = compute_safety_value(...
                    evader, pursuer, g, data);
                safety_value_matrix(i, j) = safety_value;
                xr_cell{i, j} = xr;
            end
        end
    end
end