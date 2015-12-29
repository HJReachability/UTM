function two_agent_reach_avoid_simulation()
% Target set example
% The plane should get to the target location
%
% Jennifer Shih
%% Preliminaries
addpath('..') % needed for Plane class
addpath(['/Users/jennifershih/Documents/Tomlin Research/' ...
    'hybrid_system_lab_git/helperOC']);

% load data from running target_set_ok.m
load(['Nx_50_minus_100_to_100_target_reachability.mat']);
% load data from running air3D.m
load(['/Users/jennifershih/Documents/Tomlin Research/', ...
    'hybrid_system_lab_git/hamilton_jacobian_toolbox/Examples/', ...
    'Reachability/cylinder_target_air3D.mat']);

capture_radius = 5;
speed = 5;
safety_threshold = 1.5;

% Initialize initial location and orientation of the agents
x1 = 100 * rand(1, 1);
y1 = 100 * rand(1, 1);
ang1 = 2 * pi * rand(1, 1);

while true
   x2 = 100 * rand(1, 1);
   y2 = 100 * rand(1, 1);
   if ((x2 - x1)^2 + (y2 - y1)^2) > (capture_radius + 1)^2
       break
   end
end
ang2 = 2 * pi * rand(1, 1);

% Initialize Planes
pl1 = Plane([x1 y1 wrapTo2Pi(ang1)]);
pl1.speed = speed;
pl2 = Plane([x2 y2 wrapTo2Pi(ang2)]);
pl2.speed = speed;

target_loc_1 = [80; 80; 0];
target_loc_2 = [80; 60; 0];

% Compute gradients from value function
target_costates = extractCostates(target_g, target_timeData);
costates = extractCostates(g, data);

figure;
pl1.plotPosition;
axis([-5 100 -5 100]);

%% Number of time steps and step size for simulation
tMax = 100;
dt = 0.1;
t = dt:dt:tMax;

%% Simulation
for i = 1:length(t)
    %% Compute relative states
    xr_1 = pl1.x - target_loc_1;
    xr_1(3) = wrapTo2Pi(xr_1(3));
    
    xr_2 = pl2.x - target_loc_2;
    xr_2(3) = wrapTo2Pi(xr_2(3));
    
    % Assume agent 2 is the pursuer and agent 1 is the evader
    xr = pl2.x - pl1.x;
    xr(3) = wrapTo2Pi(xr(3));
    xr(1:2) = rotate2D(xr(1:2), -pl1.x(3));
    safety_value = eval_u(g, data, xr);
    
    %% Compute controls
    if safety_value <= safety_threshold
        % agent 1 evades and agent 2 goes to the target
        u1 = compute_control_evader(g, data, costates, xr);
        u2 = compute_target_control(target_g, target_timeData, ...
            target_costates, xr_2);
    else
        % both agents go to the target
        u1 = compute_target_control(target_g, target_timeData, ...
            target_costates, xr_1);
        u2 = compute_target_control(target_g, target_timeData, ...
            target_costates, xr_2);
    end
  
    %% Update states and plot
    pl1.updateState(u1, dt); 
    pl2.updateState(u2, dt);
    pl1.plotPosition;
    pl2.plotPosition;
    drawnow;
  
    %% Test collision
    if norm(pl1.x(1:2) - pl2.x(1:2), 2) <= capture_radius
        error('Collision');
    end
    %% If inside the target set, then stops.
    if norm(pl1.x(1:2) - target_loc_1(1:2), 2) <= capture_radius && ...
        norm(pl2.x(1:2) - target_loc_2(1:2), 2) <= capture_radius
        fprintf('Both agents reached target');
        break;
    end
end
end

%% Helper functions
function u = compute_control_evader(g, data, costates, xr)
    valuex = eval_u(g, data, xr); 

    % Plane 1 computes optimal control to avoid plane 2 if unsafe
    p = calculateCostate(g, costates, xr);
    if p(1) * xr(2) - p(2) * xr(1) - p(3) >= 0
        u = 1;
    else
        u = -1;
    end
end

function u = compute_target_control(target_g, target_timeData, ... 
    target_costates, xr)

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
end