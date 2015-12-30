function [control_target_list time_to_reach_target_list] = ...
    compute_target_control_value_list(list_planes, indices_active, ...
    target_locations_cell, target_g, target_timeData, target_costates)
    
    num_planes = length(list_planes);
    control_target_list = [];
    time_to_reach_target_list = [];
    for i=1:num_planes
        index_target_cell = indices_active(i);
        target_loc = target_locations_cell{index_target_cell};
        plane_loc = list_planes(i).x(1:2);
        xr = plane_loc - target_loc;
        xr = [xr 0];
        
        valuex = eval_u(target_g, target_timeData, xr);
        if isnan(valuex)
            error('Outside of evaluation boundary for target set');
        end

        p = calculateCostate(target_g, target_costates, xr);

        % Minimize the hamiltonian
        if p(3) > 0
            u = -1;
        else
            u = 1;
        end
        control_target_list = [control_target_list u];
        time_to_reach_target_list = [time_to_reach_target_list valuex];
    end
end