function safety_value_matrix = compute_pairwise_safety_vals(list_planes, ...
        safety_threshold, g, data, costates)
%% Description
% Computes the safety value for each pair of planes
% 
%% Main function body
    numplanes = length(list_planes);
    safety_value_matrix = NaN(num_planes);
    for i=1:num_planes
        for j=1:num_planes
            if i ~= j
                evader = list_planes(i);
                pursuer = list_planes(j);
                safety_value_matrix(i, j) = compute_safety_value(...
                    evader, pursuer, g, data, costates);
            end
        end
    end
end