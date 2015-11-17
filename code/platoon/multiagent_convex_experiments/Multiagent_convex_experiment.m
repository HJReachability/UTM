function multiagent_convex_experiment()
% A function for testing different models for multi-agent collision
% avoidance with formulation of Hamiltonian Jacobian
%
% - Jennifer Shih 
%% Set up data
addpath('..') % needed for Plane class
addpath('/Users/jennifershih/Documents/Tomlin Research/hybrid_system_lab_git/helperOC');

% load the output g and data from running air3D.m in the levelset toolbox
load('/Users/jennifershih/Documents/Tomlin Research/hybrid_system_lab_git/hamilton_jacobian_toolbox/Examples/Reachability/cylinder_target_air3D.mat')
%load('../tests/Plane_test_data/air3D_data');
capture_radius = 5;
speed = 5;

% Initialize Planes
pl1 = Plane([0 20 -pi/4]);
pl1.speed = speed; % This is needed if the state is 3D!
pl2 = Plane([20 0 3*pi/4]);
pl2.speed = speed;
pl3 = Plane([0 0 pi/4]);
pl3.speed = speed;
pl4 = Plane([20 20 pi + pi/4]);
pl4.speed = speed;
%pl5 = Plane([10 10 -pi/4]);
%pl5.speed = speed;

% A list of planes
list_planes = [pl1 pl2 pl3 pl4];
num_planes = length(list_planes);

% Safety threshold settings for control
safety_threshold = 1;

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
        if norm(list_planes(j).getPosition - list_planes(k).getPosition) <= capture_radius
            error(sprintf('Collisions for plane %d and %d', j, k))
        end
     end
  end
  
  %% Compute pairings for each intersection
  %pairings = three_agent_basic_solver();
  pairings = safety_penalty_obj_solver(list_planes, safety_threshold, g, data, costates);
  %% Computes control for each agent based on pairings (evader, pursuer)
  for planeNo = 1:num_planes
    evader = list_planes(planeNo);
    pursuer = list_planes(pairings(num2str(planeNo)));
    %disp([num2str(planeNo) ' evades ' num2str(pairings(num2str(planeNo)))])
    controls(planeNo) = compute_control_evader(evader, pursuer, safety_threshold, g, data, costates);
  end
  
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

function u = compute_control_evader(evader, pursuer, safety_threshold, g, data, costates)
  xr = pursuer.x - evader.x;

  % rotate position vector so it is correct relative to xe heading
  xr(1:2) = rotate2D(xr(1:2), -evader.x(3));

  % Wrap angle to [0, 2pi]
  xr(3) = wrapTo2Pi(xr(3));

  %% Compute controls
  valuex = eval_u(g, data, xr); % Plug relative state into value function

  disp(['Safety value: ' num2str(valuex)])
  
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