function pairings = baseline_solver(list_planes)
    num_planes = length(list_planes);
    %% Computes the distance for each pair of planes
    distance_matrix = inf(num_planes);
    for i=1:num_planes
        for j=1:num_planes
            if i ~= j
                evader_state = list_planes(i).x;
                pursuer_state = list_planes(j).x;  
                distance_matrix(i, j) = norm(evader_state(1:2) - ...
                    pursuer_state(1:2), 2);
            end
        end
    end

    %% Create the mappings
    pairings = containers.Map;
    minimum_col = min(distance_matrix);
    for i=1:num_planes
        index = num2str(i);
        pursuer = find(distance_matrix(:, i) == minimum_col(i));
        pairings(index) = pursuer;
    end

end