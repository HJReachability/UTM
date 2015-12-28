function safety_value = compute_safety_value(evader, pursuer, g, ...
    data, costates)
%% Description
%  Computes the safety value for specified pair of evader and pursuer
%
%% Function main body
    xr = pursuer.x - evader.x;
    xr(1:2) = rotate2D(xr(1:2), -evader.x(3));
    xr(3) = wrapTo2Pi(xr(3));
    
    safety_value = eval_u(g, data, xr); 
end