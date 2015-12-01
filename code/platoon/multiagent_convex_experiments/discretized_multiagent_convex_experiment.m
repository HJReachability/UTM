function multiagent_convex_experiment()
% A function for testing different models for multi-agent collision
% avoidance with formulation of Hamiltonian Jacobian
%
% - Jennifer Shih 
%% Set up data
addpath('..') % needed for Plane class
addpath(['/Users/jennifershih/Documents/Tomlin Research/' ...
    'hybrid_system_lab_git/helperOC']);

% load the output g and data from running air3D.m in the levelset toolbox
load(['/Users/jennifershih/Documents/Tomlin Research/', ...
    'hybrid_system_lab_git/hamilton_jacobian_toolbox/Examples/', ...
    'Reachability/cylinder_target_air3D.mat']);

%load('../tests/Plane_test_data/air3D_data');
capture_radius = 5;
speed = 5;

pl1 = Plane([0 18 -pi/4]);
pl1.speed = speed; 
pl2 = Plane([15 0 pi/2]);
pl2.speed = speed;
pl3 = Plane([18 18 5.0/4 * pi]);
pl3.speed = speed;

% A list of planes
list_planes = [pl1 pl2 pl3];
num_planes = length(list_planes);

% Safety threshold settings for control
safety_threshold = 1.0;

% Compute gradients from value function
costates = extractCostates(g, data);

figure;
for i=1:num_planes
    list_planes(i).plotPosition;
end
axis([-5 25 -5 25]);

% Integration parameters
tMax = 5;
dt = 0.1;
t = dt:dt:tMax;

controls = zeros(num_planes, 1);

for i = 1:length(t)
    %% Detect collision 
    for j=1:num_planes-1
        for k=j+1:num_planes
            if norm(list_planes(j).getPosition - list_planes(k).getPosition)...
                    <= capture_radius
                error(sprintf('Collisions for plane %d and %d', j, k))
            end
        end
    end
  
    %% Compute pairings for each intersection
    controls = discretization_solver(list_planes, safety_threshold, g, data, costates, dt);
    %% Update states and plot
    for planeNo = 1:num_planes
        u = controls(planeNo);
        list_planes(planeNo).updateState(u, dt);
    end
  
    %% Plot the planes
    for planeNo = 1:num_planes
        list_planes(planeNo).plotPosition;
    end
  
    drawnow;

    end
end