function planes_to_avoid = compute_planes_to_avoid_dict(num_planes, x)
    %% Take in solution from the binary linear program and compute
    % a dictionary where key i maps to a list of agents that agent i
    % should try to avoid
    %% Function body
    planes_to_avoid = containers.Map;
    for i=1:num_planes
        planes_to_avoid(num2str(i)) = [];
    end
    for i=1:num_planes
        index = (i-1) * (num_planes - 1);
        for j=1:num_planes - 1
            if j < i 
                if x(index + j) == 1
                    planes_to_avoid(num2str(i)) = ...
                        [planes_to_avoid(num2str(i)) j];
                end
            else       
                if x(index + j) == 1
                    planes_to_avoid(num2str(i)) = ...
                        [planes_to_avoid(num2str(i)) j+1];
                end
            end
        end
    end
end