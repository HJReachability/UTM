function success = simulate_new_method(N)
%% Simulate the new method
%
% N = number of planes
%
% Jennifer Shih   

%% Preliminaries
addpath('..') % needed for Plane class
addpath('./utils');
addpath('./solvers');
addpath(['/Users/jennifershih/Documents/Tomlin Research/' ...
    'hybrid_system_lab_git/helperOC']);

% load data from running target_set_ok.m
load(['data/Nx_50_minus_100_to_100_target_reachability.mat']);
% load data from running air3D.m
load(['/Users/jennifershih/Documents/Tomlin Research/', ...
    'hybrid_system_lab_git/hamilton_jacobian_toolbox/Examples/', ...
    'Reachability/cylinder_target_air3D.mat']);

success = true;

capture_radius = 5;
speed = 5;
board_size = 100; 

%% Generate random starting locations for each plane and initialize 
% each plane
all_planes = ...
    initialize_plane_locations(N, board_size, capture_radius, speed)

%% Plot planes
for i=1:N
    all_planes(i).plotPosition;
end
axis([-5 board_size -5 board_size]);

%% Generate target locations (not random) and plot locations
% all_target_locations is a Nx1 cell
all_target_locations = generate_target_locations(N, board_size); 
for i=1:N
    loc = all_target_locations{i};
    plot(loc(1), loc(2),'o','MarkerSize',10);
end

%% Start simulation
%% Number of time steps and step size for simulation
tMax = 100;
dt = 0.1;
t = dt:dt:tMax;

%% Compute gradients from value function
target_costates = extractCostates(target_g, target_timeData);
costates = extractCostates(g, data);

%% Set safety_threshold
safety_threshold = 1.5;

%% Construct indices for planes that have not yet reached target
indices_active = 1:N; % Needs to be sorted
active_planes = all_planes;
num_active_planes = N;

%% Simulation
for i = 1:length(t)
    
    %% Compute safety matrix
    [safety_matrix xr_cell] = compute_pairwise_safety_vals(active_planes, ...
        safety_threshold, g, data);
    
    %% Compute control matrix
    control_matrix = compute_pairwise_control(num_active_planes, ...
        safety_matrix, xr_cell, safety_threshold, g, costates);
    
    %% Compute planes to avoid dictionary based on the safety and control
    % matrix above
    planes_to_avoid = optimal_simple_solver_extension(num_active_planes, ...
        safety_matrix, safety_threshold, control_matrix);
    
    %% Compute time to target value and control to reach target for active
    % planes
    [control_target_list time_to_reach_target_list] = 
        compute_target_control_value_list(list_planes, indices_active, ...
        all_target_locations, target_g, target_timeData, target_costates)
    
    %% Compute control for active planes
    controls = determine_control_after_simple_solver_extension(...
        planes_to_avoid, safety_matrix, safety_threshold, control_matrix, ...
        control_target_list, time_to_reach_target_list)
    
    %% Update states and plot
    for i=1:length(active_planes)
        active_planes(i).updateState(controls(i), dt);
    end
    for i=1:length(active_planes)
        active_planes(i).plotPosition();
    end
    drawnow;
  
    %% Test collision
    for i=1:length(active_planes)-1
        state_i = active_planes(i).x;
        for j=i+1:length(active_planes)
            state_j = active_planes(j).x;
            if norm(state_i(1:2) - state_j(1:2), 2) <= capture_radius
                fprintf('Collision: %d and %d \n', indices_active(i), ...
                    indices_active(j));
                success = false; 
            end
        end
    end

    %% Remove plane from the plane list if inside target set
    keep_indices = [];
    for i=1:length(active_planes)
        state = active_planes(i).x;
        if norm(state(1:2) - all_targets{indices_active(i)}, 2) ...
                > capture_radius
            keep_indices = [keep_indice i];
        end
    end
    indices_active = indices_active(keep_indices);
    active_planes = all_planes(indices_active);

    %% Exit program when all planes have reached target
    if length(active_planes) == 0
       break; 
    end
end
    
end