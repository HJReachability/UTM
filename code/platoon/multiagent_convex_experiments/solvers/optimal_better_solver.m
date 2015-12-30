function planes_to_avoid = optimal_better_solver(num_planes, control_matrix)
%% 
% This function solves the optimization problem 
%
% max \sum_{i} \abs{\sum_{j} a_{ij}u_ij} - \sum_{i, j}u_{ij}
% subject to
%   abs{a_{ij}}u_{ij} + abs{a_{ji}}u_{ji} = max(abs{a_{ij}}, abs{a_{ji}})
%   u_{ij} \in {0, 1}
%
% where a_{ij} = control_matrix(i, j) 
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
assert(num_planes == length(control_matrix));

%% Parameter setup for generalization to n airplanes
%names = {'u_12'; 'u_13'; 'u_21'; 'u_23'; 'u_31'; 'u_32'};

N_ineq = 2 * num_planes;
N_eq = num_planes * (num_planes - 1) / 2; % number of equalities
N_var_u = num_planes * (num_planes - 1); % number of variables u_ij
N_var_v = num_planes; % number of variables v_i
N_var = N_var_u + N_var_v; % total number of variables

% construct equality and inequality signs
sense = strcat(repmat(sprintf('='), 1, N_eq), ...
    repmat(sprintf('<'), 1, N_ineq));

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
for i=1:num_planes
    index = N_var_u + i;
    names{index} = strcat('v_', num2str(i));
end

% Construct obj
obj = [-ones(1, N_var_u) ones(1, N_var_v)];

% Construct equality constraints (Ax = rhs)
rhs = nan(N_eq + N_ineq, 1);
A = zeros(N_eq + N_ineq, N_var);
index = 1;
for i = 1:num_planes-1
    for j=i+1:num_planes
        rhs(index) = max(abs(control_matrix(i, j)), ...
            abs(control_matrix(j, i)));
        col_index_1 = (i - 1) * (num_planes - 1) + (j - 1);
        col_index_2 = (j - 1) * (num_planes - 1) + i;
        A(index, col_index_1) = abs(control_matrix(i, j));
        A(index, col_index_2) = abs(control_matrix(j, i));
        index = index + 1;
    end
end

for i=1:num_planes
    index = (i-1) * (num_planes - 1);
    vert_index = (i-1) * 2;
    for j=1:num_planes 
        if i ~= j
            if i < j
                A(N_eq + vert_index + 1, index + j - 1) = ...
                    control_matrix(i, j);
                A(N_eq + vert_index + 2, index + j - 1) = ...
                    -control_matrix(i, j);
            else
                A(N_eq + vert_index + 1, index + j) = control_matrix(i, j);
                A(N_eq + vert_index + 2, index + j) = -control_matrix(i, j);
            end
        end
    end
    A(N_eq + vert_index + 1, N_var_u + i) = -1;
    A(N_eq + vert_index + 2, N_var_u + i) = -1;
    rhs(N_eq + vert_index + 1) = 0;
    rhs(N_eq + vert_index + 2) = 0;
end

tmp = 1;

%% Feed into gurobi solver 
try
    clear model;
    model.obj = obj;
    model.A = sparse(A);
    model.rhs = rhs;
    model.sense = sense;
    % https://www.gurobi.com/documentation/6.5/examples.pdf, p497 example
    model.vtype = [repmat('B', N_var_u, 1); repmat('I', N_var_v, 1)];
    model.modelsense = 'max';
    model.varnames = names;

    clear params;
    params.outputflag = 0;

    result = gurobi(model, params);

    disp(result)

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