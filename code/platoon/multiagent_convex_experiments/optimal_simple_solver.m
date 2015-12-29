function planes_to_avoid = optimal_simple_solver(num_planes, ...
    safety_value_matrix, safety_threshold)
%% 
% This function solves the optimization problem 
%
% min \sum_{i, j} u_{ij}
% subject to
%   a_{ij}u_{ij} + a_{ji}u_{ji} = max(a_{ij}, a_{ji})
%   u_{ij} \in {0, 1}
%
% where a_{ij} = 1 if safety_value_matrix(i, j) <= safety_threshold and
% a_{ij} = 0 otherwise
%
% Input - 
%   safety_value_matrix: consists of pairwise safety value for each plane
%   safety_threshold: a scalar representing the safety_threshold
% Output - 
%   avoidance_matrix - avoidance_matrix(i, j) = 1 if agent i should try to
%                      avoid agent j
% Jennifer Shih
%
%% Main function body 
assert(num_planes == length(safety_value_matrix));

safety_value_binary = (safety_value_matrix <= safety_threshold);
% Note that comparison of a number with NaN is always 0

%% Parameter setup for generalization to n airplanes
%names = {'u_12'; 'u_13'; 'u_21'; 'u_23'; 'u_31'; 'u_32'};

N_eq = num_planes * (num_planes - 1) / 2; % number of equalities
N_var = num_planes * (num_planes - 1); % number of variables
sense = repmat(sprintf('='), 1, N_eq); % construct equality signs

% Construct the names (u_{i}_{j})
names = cell(N_var, 1);
for i=1:num_planes
    for j=1:num_planes
        if i ~= j
            if i < j
                index = (i-1) * (num_planes - 1) + j - 1;
            else
                index = (i-1) * (num_planes - 1) + j;
            end
            names{index} = strcat('u_', num2str(i), '_', num2str(j));
        end
    end
end

% Construct obj
obj = ones(1, N_var);

% Construct equality constraints (Ax = rhs)
rhs = nan(N_eq, 1);
A = zeros(N_eq, N_var);
index = 1;
for i = 1:num_planes-1
    for j=i+1:num_planes
        rhs(index) = max(safety_value_binary(i, j), ...
            safety_value_binary(j, i));
        col_index_1 = (i - 1) * (num_planes - 1) + (j - 1);
        col_index_2 = (j - 1) * (num_planes - 1) + i;
        A(index, col_index_1) = safety_value_binary(i, j);
        A(index, col_index_2) = safety_value_binary(j, i);
        index = index + 1;
    end
end

%% Feed into gurobi solver 
try
    clear model;
    model.obj = obj;
    model.A = sparse(A);
    model.rhs = rhs;
    model.sense = sense;
    model.vtype = 'B';
    model.modelsense = 'min';
    model.varnames = names;

    clear params;
    params.outputflag = 0;

    result = gurobi(model, params);

    %disp(result)

    for v=1:length(names)
        fprintf('%s %d\n', names{v}, result.x(v));
    end

    fprintf('Obj: %e\n', result.objval);
    
    % Construct pairings
    % A dictionary key: agent i, value: the agents it should try to avoid
    planes_to_avoid = containers.Map;
    for i=1:num_planes
        planes_to_avoid(num2str(i)) = [];
    end
    for i=1:num_planes
        index = (i-1) * (num_planes - 1);
        for j=1:num_planes - 1
            if j < i 
                if result.x(index + j) == 1
                    planes_to_avoid(num2str(i)) = ...
                        [planes_to_avoid(num2str(i)) j];
                end
            else       
                if result.x(index + j) == 1
                    planes_to_avoid(num2str(i)) = ...
                        [planes_to_avoid(num2str(i)) j+1];
                end
            end
        end
    end

catch gurobiError
    fprintf('Error reported\n');
end
end