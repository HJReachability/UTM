% This file generates random data for control and safety matrices
% control_matrix(i, j) takes on value in {-1, 0, 1}
% 
% control_matrix(i, j) = 1 if optimal control for agent i to avoid agent
% j is 1, -1 if optimal control for agent i to avoid agent j is -1, and
% 0 if agent i does not need to avoid agent j

function [safety_matrix control_matrix] = ...
    generate_random_control_and_safety_matrix(N, safety_threshold)
    control_matrix = nan(N);
    safety_matrix = nan(N);
    eps = 0.01;
    for i=1:N
        for j=1:N
            if i ~= j
               if rand(1, 1) >= 2/3
                   control_matrix(i, j) = 0;
                   safety_matrix(i, j) = safety_threshold + eps;
               elseif rand(1, 1) >= 1/3
                   control_matrix(i, j) = 1;
                   safety_matrix(i, j) = safety_threshold - eps;
               else
                   control_matrix(i, j) = -1;
                   safety_matrix(i, j) = safety_threshold - eps;
               end    
            end
        end
    end
end




