function controls = discretization_solver(list_planes, safety_threshold, g, data, costates, dt)
    %%
    % A function that computes the best control for each agent by
    % discretizing the control and doing a one step lookahead
    %
    % Jennifer Shih
    
    discretization_step = 1;
    u_discretized = -1:discretization_step:1;
    num_planes = length(list_planes);
    num_discretized = length(u_discretized);
    num_states = list_planes(1).nx;
    
    %% Compute the state of each plane after applying each control
    states = zeros(num_planes, num_discretized, num_states);
    
    for i = 1:num_planes
        plane = list_planes(i);
        for j = 1:num_discretized
            u = u_discretized(j);
            states(i, j, :) = plane.computeState(u, dt);
        end
    end
    
    %% Form a matrix containing the safety value for each pair of the controls
    safety_dict = containers.Map;

    % Initialization
    for i=1:num_planes
        for j=1:num_planes
            if i ~= j
                key = [num2str(i) num2str(j)];
                safety_mat = zeros(num_discretized);
                for a = 1:num_discretized
                    for b = 1:num_discretized
                        evader_state = states(i, a, :);
                        pursuer_state = states(j, b, :);
                        evader_state = reshape(evader_state, [1, 3]);
                        pursuer_state = reshape(pursuer_state, [1, 3]);
                        safety_mat(a, b) = compute_safety_value(evader_state, pursuer_state, g, data, costates)
                    end
                end
                safety_dict(key) = safety_mat;
            end
        end
    end
    
    %% Optimization problem -- parameter setup
%% Parameter setup for generalization to n airplanes
% names of planes: u_1_1, u_1_2, ..., u_1_N, u_2_1, ..., u_2_N, u_3_1, ...,
% u_3_N

% N = num_planes * num_discretized;
% rhs = ones(num_planes, 1);
% sense = repmat(sprintf('='), 1, num_planes);
% 
% % Construct the names
% names = cell(N, 1);
% for i=1:num_planes
%     for j=1:num_discretized
%            index = (i - 1) * num_discretized + j;
%            names{index} = strcat('u_', num2str(i), '_', num2str(j));
%         end
%     end
% end
% 
% % Construct the first portion of A
% A = zeros(num_planes, N);
% ones_row = ones(1, num_discretized);
% for i=1:num_planes
%     base_index = (i - 1) * num_discretized;
%     A(i, base_index+1:base_index+num_discretized) = ones_row;
% end

min_objective = inf;
controls = NaN;
beta = 0.5;
alpha = 0.1;
epsilon = 0.5;
objectives = zeros(num_discretized, num_discretized, num_discretized);
%% Brute force the problem
for i=1:num_discretized
    for j=1:num_discretized
        for k=1:num_discretized
            safety_matrix = zeros(num_planes);
            % i corresponds to control index for plane 1
            % j corresponds to control index for plane 2
            % k corresponds to control index for plane 3
            % Compute the objective after plane 1, 2, 3 executes its 
            % respective control

            % Construct the safety matrix
            dict = safety_dict('12');
            safety_matrix(1, 2) = dict(i, j);
            dict = safety_dict('21');
            safety_matrix(2, 1) = dict(j, i);
            dict = safety_dict('13');
            safety_matrix(1, 3) = dict(i, k);
            dict = safety_dict('31');
            safety_matrix(3, 1) = dict(k, i);
            dict = safety_dict('23');
            safety_matrix(2, 3) = dict(j, k);
            dict = safety_dict('32');
            safety_matrix(3, 2) = dict(k, j);
            
            objective = 0;
            % Compute objective based on safety matrix
            %   \sum_{ij} (safety_threshold - safety_value)_+^2 + 
            %              \alpha \sum_{ij} u_{ij} (safety_threshold + epsilon - 
            %              safety_value)_+^2 
            matrix_penalize_0 = 0 - safety_matrix;
            matrix_penalize_0(isnan(matrix_penalize_0)) = 0;
            
            matrix_penalize_1 = safety_threshold - safety_matrix;
            matrix_penalize_1(isnan(matrix_penalize_1)) = 0;
            matrix_penalize_2 = safety_threshold + epsilon - safety_matrix;
            matrix_penalize_2(isnan(matrix_penalize_2)) = 0;
            
            matrix_penalize_0(matrix_penalize_0 < 0) = 0;
            matrix_penalize_1(matrix_penalize_1 < 0) = 0;
            matrix_penalize_2(matrix_penalize_2 < 0) = 0;
            matrix_penalize_0 = matrix_penalize_0.^4;
            matrix_penalize_1 = matrix_penalize_1.^2;
            matrix_penalize_2 = matrix_penalize_2;
            objective = sum(sum(matrix_penalize_1)) + alpha * sum(sum(matrix_penalize_2));
            objectives(i, j, k) = objective;
            
            if objective < min_objective
                min_objective = objective;
                controls = [u_discretized(i) u_discretized(j) u_discretized(k)];
            end
        end
    end
end

end


function safety_value = compute_safety_value(evader_state, pursuer_state, g, data, costates)
    xr = pursuer_state - evader_state;
    xr(1:2) = rotate2D(xr(1:2), - evader_state(3));
    xr(3) = wrapTo2Pi(xr(3));
    
    safety_value = eval_u(g, data, xr); 
end