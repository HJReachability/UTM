% This function formulates and solves the following model:
% The model assumes each agent avoids exactly one other agent at any given
% time. The objective seeks to minimize the cost_danger(u)
%  maximize (over u_ij)
%        \sum_{ij} u_{ij} (safety_threshold - safety_value)_+^2 + 
%              \alpha \sum_{ij} u_{ij} (safety_threshold + epsilon - 
%              safety_value)_+^2
%  subject to
%        \sum_{j}u_ij = 1 for all i
%        all variables are binary
%
% Jennifer Shih

function pairings = safety_penalty_obj_solver(list_planes, safety_threshold, g, data, costates)
 num_planes = length(list_planes);
 %% Computes the safety value for each pair of planes
 safety_value_matrix = NaN(num_planes);
 for i=1:num_planes
     for j=1:num_planes
         if i ~= j
            evader = list_planes(i);
            pursuer = list_planes(j);
            safety_value_matrix(i, j) = compute_safety_value(evader, pursuer, g, data, costates);
         end
     end
 end
 
 disp(safety_value_matrix);
 %% Fill in the values that are NaN with the inverse distance between the two planes
 % (Can be combined with the above for loop)
%   dist_const = 1.0; 
%   for i=1:num_planes
%      for j=1:num_planes
%          if i ~= j
%             evader = list_planes(i);
%             pursuer = list_planes(j);
%             if isnan(safety_value_matrix(i, j))
%                 safety_value_matrix(i, j) = dist_const/norm(evader.getPosition - pursuer.getPosition);
%             end
%          end
%      end
%   end

%% Construct objective value
alpha = 0.01;
epsilon = 6.0;

vec1 = zeros(1, num_planes * (num_planes - 1));
vec2 = zeros(1, num_planes * (num_planes - 1));
index = 0;
for i=1:num_planes
     for j=1:num_planes
         if i ~= j
            index = index + 1;
            evader = list_planes(i);
            pursuer = list_planes(j);
            if ~isnan(safety_value_matrix(i, j))
                vec1(index) = safety_threshold - safety_value_matrix(i, j);
                vec2(index) = safety_threshold + epsilon - safety_value_matrix(i, j);
            end
         end
     end
end
vec1((vec1 < 0)) = 0;
vec2((vec2 < 0)) = 0;
vec1 = vec1.^2;
vec2 = vec2.^2;
vec2 = alpha * vec2;

disp(vec1);
disp(vec2);

%% Parameter setup for generalization to n airplanes
%names = {'u_12'; 'u_13'; 'u_21'; 'u_23'; 'u_31'; 'u_32'};

N = num_planes * (num_planes - 1);
rhs = ones(num_planes + N/2, 1);
rhs(num_planes + 1: num_planes + N/2) = 2;
sense = strcat(repmat(sprintf('='), 1, num_planes), repmat(sprintf('<'), 1, N/2));

% Construct the names
names = cell(N, 1);
for i=1:num_planes
    for j=1:num_planes
        if i ~= j
            if i < j
                index = (i-1) * (num_planes - 1) + j - 1;
            else
                index = (i-1) * (num_planes - 1) + j;
            end
            names{index} = strcat('u_', num2str(i), num2str(j));
        end
    end
end

% Construct the first portion of A
A = zeros(length(rhs), N);
for i=1:num_planes
    index = (i - 1) * (num_planes - 1);
    A(i, index + 1:index + num_planes - 1) = 1;
end

% Construct the second portion of A 
base_index = num_planes;
index = 0;
for i=1:num_planes-1
    for j=i+1:num_planes
        index = index+1;
        y1 = (i - 1) * (num_planes - 1) + j-1;
        y2 = (j - 1) * (num_planes - 1) + i;
        x = base_index + index;
        A(x, y1) = 1;
        A(x, y2) = 1;
    end
end

%% Feed into gurobi solver with objective different objective
try
    clear model;
    model.obj = vec1 + vec2;
    %model.A = sparse([1 1 0 0 0 0; 0 0 1 1 0 0; 0 0 0 0 1 1; 1 0 1 0 0 0; 0 1 0 0 1 0; 0 0 0 1 0 1]);
    model.A = sparse(A);
    model.rhs = rhs;
    model.sense = sense;
    model.vtype = 'B';
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
    
    % Return the pairings 
    % A dictionary key: agent i, value: the agent it tries to avoid
    pairings = containers.Map;
    for i=1:num_planes
        index = (i-1) * (num_planes - 1);
        for j=1:num_planes - 1
            if j < i 
                if result.x(index + j) == 1
                    pairings(num2str(i)) = j;
                    break;
                end
            else       
                if result.x(index + j) == 1
                    pairings(num2str(i)) = j+1;
                    break;
                end
            end
        end
    end

catch gurobiError
    fprintf('Error reported\n');
end
end

function safety_value = compute_safety_value(evader, pursuer, g, data, costates)
    xr = pursuer.x - evader.x;
    xr(1:2) = rotate2D(xr(1:2), -evader.x(3));
    xr(3) = wrapTo2Pi(xr(3));
    
    safety_value = eval_u(g, data, xr); 
end