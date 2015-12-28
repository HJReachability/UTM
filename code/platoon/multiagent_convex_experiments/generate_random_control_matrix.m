% This file generates random data for control 
% control_matrix(i, j) takes on value in {-1, 0, 1}
% 
% control_matrix(i, j) = 1 if optimal control for agent i to avoid agent
% j is 1, -1 if optimal control for agent i to avoid agent j is -1, and
% 0 if agent i does not need to avoid agent j

function control_matrix = generate_random_control_matrix(N)
    control_matrix = nan(N);
    for i=1:N
        for j=1:N
            if i ~= j
               if rand(1, 1) >= 2/3
                   control_matrix(i, j) = 0;
               elseif rand(1, 1) >= 1/3
                   control_matrix(i, j) = 1;
               else
                   control_matrix(i, j) = -1;
               end    
            end
        end
    end
end




