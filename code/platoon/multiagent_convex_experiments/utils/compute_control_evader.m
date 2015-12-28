function u = compute_control_evader(evader, pursuer, safety_threshold, ...
    g, data, costates)
    %% Compute safety value
    valuex = compute_safety_value(evader, pursuer, g, data, costates);
  
    %% Compute control
    if valuex <= safety_threshold
        % Plane 1 computes optimal control to avoid plane 2 if unsafe
        p = calculateCostate(g, costates, xr);
        if p(1) * xr(2) - p(2) * xr(1) - p(3) >= 0
            u = 1;
        else
            u = -1;
        end
    else
        % Evader goes straight if it is in a safe region
        u = 0;
    end
end