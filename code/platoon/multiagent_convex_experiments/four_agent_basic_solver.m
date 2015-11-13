% This example formulates and solves the following simple MIP model:
%  maximize (over u_12, u_13, u_23, u_32, u_13, u_31, u_41, u_42, u_43)
%        0
%  subject to
%        \sum_{j}u_ij = 1
%  
%  all variables are binary
%
% Jennifer Shih
function pairings = four_agent_basic_solver()

names = {'u_12'; 'u_13'; 'u_14'; 'u_21'; 'u_23'; 'u_24'; 'u_31'; 'u_32'; 'u_34'; 'u_41'; 'u_42'; 'u_43'};
num_agents = 4;

try
    clear model;
    model.obj = zeros(1, 12);
    A = zeros(4, 12);
    A(1, 1) = 1;
    A(1, 2) = 1;
    A(1, 3) = 1;
    A(2, 4) = 1;
    A(2, 5) = 1;
    A(2, 6) = 1;
    A(3, 7) = 1;
    A(3, 8) = 1;
    A(3, 9) = 1;
    A(4, 10) = 1;
    A(4, 11) = 1;
    A(4, 12) = 1;
    model.A = sparse(A);
    model.rhs = [1; 1; 1; 1];
    model.sense = '====';
    model.vtype = 'B';
    model.modelsense = 'max';
    model.varnames = names;

    gurobi_write(model, '4_agents_mip.lp');

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
    for i=1:num_agents
        index = (i-1) * (num_agents - 1);
        for j=1:num_agents-1
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
