% This example formulates and solves the following simple MIP model:
%  maximize (over u_12, u_13, u_23, u_32, u_13, u_31
%        0
%  subject to
%        \sum_{j}u_ij = 1
%        u_ij + u_ji \geq 1
%
%  all variables are binary
%
% Jennifer Shih
function pairings = three_agent_basic_solver()

names = {'u_12'; 'u_13'; 'u_21'; 'u_23'; 'u_31'; 'u_32'};
num_agents = 3;

try
    clear model;
    model.obj = zeros(1, 6);
    model.A = sparse([1 1 0 0 0 0; 0 0 1 1 0 0; 0 0 0 0 1 1; 1 0 1 0 0 0; 0 1 0 0 1 0; 0 0 0 1 0 1]);
    model.rhs = [1; 1; 1; 1; 1; 1];
    model.sense = '===>>>';
    model.vtype = 'B';
    model.modelsense = 'max';
    model.varnames = names;

    gurobi_write(model, '3_agents_mip.lp');

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
