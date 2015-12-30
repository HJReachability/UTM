function planes_to_avoid = optimal_simple_solver_extension(num_planes, ...
    safety_value_matrix, safety_threshold, control_matrix)
%% 
% This function provides approximate solution to the optimization problem 
%
% max \sum_{i} \abs{\sum_{j} a_{ij}u_ij} - \sum_{i, j}u_{ij}
% subject to
%   abs{a_{ij}}u_{ij} + abs{a_{ji}}u_{ji} = max(abs{a_{ij}}, abs{a_{ji}})
%   u_{ij} \in {0, 1}
%
% where a_{ij} = control_matrix(i, j) 
%
% Jennifer Shih
%% Main function body
    assert(num_planes == length(control_matrix));
    assert(num_planes == length(safety_value_matrix));

    all_possible_solutions = optimal_simple_solver_multiple(num_planes, ...
    safety_value_matrix, safety_threshold);

    control_matrix_flattened = [];
    for i=1:num_planes
        for j=1:num_planes
            if i ~= j
                control_matrix_flattened = [control_matrix_flattened ...
                    control_matrix(i, j)];
            end
        end
    end
    
    curr_max = -inf;
    max_sol = nan;
    for i=1:size(all_possible_solutions, 1)
        obj = compute_objective(num_planes, control_matrix_flattened, ...
            all_possible_solutions(i, :));
        if obj > curr_max
           curr_max = obj;
           max_sol = all_possible_solutions(i, :);
        end
    end

    % Compute dictionary
    planes_to_avoid = compute_planes_to_avoid_dict(num_planes, max_sol);
end

function total = compute_objective(N, control_matrix_flattened, solution)
    total = 0;
    for i = 1:N
        base = (i - 1) * (N - 1);
        total = total + abs(sum(control_matrix_flattened(base+1:base+N-1) ...
            .* solution(base+1:base+N-1)));
    end
end