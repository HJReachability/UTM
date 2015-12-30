% This file generates the data for saftey value testing

function [safety_matrix safety_threshold] = generate_random_safety_matrix(N)
    safety_threshold = 1.5;
    safety_matrix = nan(N);

    for i=1:N
        for j=1:N
            if i ~= j
               if rand(1, 1) >= 0.5
                   safety_matrix(i, j) = safety_threshold + 0.1;
               else
                   safety_matrix(i, j) = safety_threshold - 0.1;
               end    
            end
        end
    end
end




