function reach_target()
% Target set example
% The plane should get to the target location
%
% Jennifer Shih
%% Preliminaries
addpath('..') % needed for Plane class
addpath(['/Users/jennifershih/Documents/Tomlin Research/' ...
    'hybrid_system_lab_git/helperOC']);

% load data from running target_set_ok.m
load(['data/Nx_50_minus_100_to_100_target_reachability.mat']);

capture_radius = 5;
speed = 5;

x = 100 * rand(1, 1);
y = 100 * rand(1, 1);
ang = 2 * pi * rand(1, 1);

% Initialize Planes
pl1 = Plane([x y wrapTo2Pi(ang)]);
pl1.speed = speed;

target_loc = [100; 100; 0];

% Compute gradients from value function
target_costates = extractCostates(target_g, target_timeData);

figure;
pl1.plotPosition;
axis([-5 100 -5 100]);

%% Number of time steps and step size for simulation
tMax = 100;
dt = 0.1;
t = dt:dt:tMax;

%% Simulation
for i = 1:length(t)
    xr = pl1.x - target_loc;
    xr(3) = wrapTo2Pi(xr(3));

    %% Compute controls
    valuex = eval_u(target_g, target_timeData, xr);
    if isnan(valuex)
        error('Outside of evaluation boundary for target set');
    end

    p = calculateCostate(target_g, target_costates, xr);
    
    % Minimize the hamiltonian
    if p(3) > 0
        u1 = -1;
    else
        u1 = 1;
    end
  
    %% Update states and plot
    pl1.updateState(u1, dt);
  
    pl1.plotPosition;
  
    drawnow;
  
    %% If inside the target set, then stops.
     if norm(pl1.x(1:2) - target_loc(1:2), 2) <= capture_radius
         fprintf('Reached target');
         disp(norm(pl1.x(1:2) - target_loc(1:2), 2));
         break;
     end
end
end