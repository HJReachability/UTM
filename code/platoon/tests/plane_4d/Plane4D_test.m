function Plane4D_test()
% Test whether 4D plane state works. uses same computations as quadrotor
%
% In this example, plane 2 always goes straight, and plane 1 avoids plane
% 2 if needed
%
% Mahesh Vashishtha, 2015-11-04

%% Preliminaries
addpath('../..') % needed for Plane class

% load the output g and data from running air3D.m in the levelset toolbox

capture_radius = 5;
speed = 5;

% Initialize Planes
pl1 = Plane([0 20 -pi/4]);
pl1.speed = speed; % This is needed if the state is 3D!
pl2 = Plane([20 0 3*pi/4]);
pl2.speed = speed;
safety_threshold = 1;

%% Join platoon / merge onto highway for quadrotors
x = [0 0 0 0]; % Base reachable set assumes 0 relative state

filename = [fileparts(mfilename('fullpath')) ...
  '/../../RS_core/saved/qr_rel_target_V.mat'];

if exist(filename, 'file')
  load(filename)
else
  [grids, datas, tau] = quad_rel_target_2D(x, visualize);

  gridLim = ...
    [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
  [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

  g = TTR_out.g;
  data = TTR_out.value;
  grad = TTR_out.grad;

  save(filename, 'g', 'data', 'grad', 'tau')
end

obj.qr_rel_target_V.g = g;
obj.qr_rel_target_V.data = data;
obj.qr_rel_target_V.grad = grad;
obj.qr_rel_target_V.tau = tau;

% Compute gradients from value function
costates = extractCostates(g, data);

figure;
pl1.plotPosition;
pl2.plotPosition;
axis([-5 25 -5 25]);

% Integration parameters
tMax = 5;
dt = 0.1;
t = dt:dt:tMax;

for i = 1:length(t)
  %% Check if the planes have collided (should not happen!)
  if norm(pl1.getPosition - pl2.getPosition) <= capture_radius
    error('Planes collided!')
  end
  
  %% Get relative state of planes to plug into the value function
  xr = pl2.x - pl1.x;

  % rotate position vector so it is correct relative to xe heading
  xr(1:2) = rotate2D(xr(1:2), -pl1.x(3));

  % Wrap angle if needed
  if xr(3) >= 2*pi
    xr(3) = xr(3) - 2*pi;
  end

  if xr(3) < 0
    xr(3) = xr(3) + 2*pi;
  end

  %% Compute controls
  valuex = eval_u(g, data, xr); % Plug relative state into value function
  disp(['Safety value: ' num2str(valuex)])
  
  % Plane 2 goes straight
  u2 = 0;
  if valuex <= safety_threshold
    % Plane 1 computes optimal control to avoid plane 2 if unsafe
    p = calculateCostate(g, costates, xr);
    if p(1) * xr(2) - p(2) * xr(1) - p(3) >= 0
      u1 = 1;
    else
      u1 = -1;
    end

  else
    % Plane 1 goes straight if safe
    u1 = 0;
  end
  
  %% Update states and plot
  pl1.updateState(u1, dt);
  pl2.updateState(u2, dt);
  
  pl1.plotPosition;
  pl2.plotPosition;
  
  drawnow;
end
end