function control_matrix = compute_pairwise_control(N, safety_value_matrix, ...
    xr_cell, safety_threshold, g, costates)
    control_matrix = nan(N);
    for i=1:N
        for j=1:N
            if i ~= j
                if safety_value_matrix(i, j) <= safety_threshold
                    xr = xr_cell{i, j};
                    p = calculateCostate(g, costates, xr);
                    if p(1) * xr(2) - p(2) * xr(1) - p(3) >= 0
                        control_matrix(i, j) = 1;
                    else
                        control_matrix(i, j) = -1;
                    end
                else
                    control_matrix(i, j) = 0;
                end
            end
        end
    end
end