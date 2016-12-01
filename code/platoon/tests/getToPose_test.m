function getToPose_test()
% getToPose_test()
%
% Tests the getToPose function in TFM and in Quadrotor (mostly TFM)
%
% Repeats the following N times:
%   Randomly assigns a target position target_dist units away from the
%   origin using a random angle (pos_theta), as well as a random target
%   heading (vel_theta). A quadrotor starts from the origin and attempts to
%   head to the target position and velocity.
%
%  If debug mode is on (debug = 1), position-velocity slices of the value
%  function in both the x and y directions, as well as the trajectories,
%  are plotted.
%
% Mo Chen, 2015-11-12

addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('qr_abs_target_V');

% Test N times...
N = 10;
for i = 1:N
  getToPoseSingle(tfm);
end
end

function getToPoseSingle(tfm)
% getToPoseSingle(tfm)
%
% A single random simulation

%% Preliminaries
debug = 1; % Debug mode
target_dist = 50; % Target distance

% Random position and heading
pos_theta = 2*pi*rand;
target_heading = pos_theta + rand*pi/3;

% Add quadrotor to TFM
init_x = zeros(4,1);
tfm.aas = {};
tfm.regVehicle(UTMQuad4D(init_x));

% Target state (random angle at 50 distance away; angle may be different
% from heading)
target_position = rotate2D([target_dist 0], pos_theta);

%% Display target state and initialize plot
f = figure;
if debug
  subplot(1,3,1)
end
quiver(target_position(1), target_position(2), ...
  10*cos(target_heading), 10*sin(target_heading), 'rx')
grid on
xlim(1.2*target_dist*[-1 1])
ylim(1.2*target_dist*[-1 1])
axis square

if debug
  f.Position(3:4) = [1000 400];
end

hold on

% Plot initial setup
level = tfm.rtt; % Level of the value function to show
%% Main plot showing position and reachable set in position space
tfm.aas{1}.plotPosition('b');
tfm.aas{1}.plot_abs_target_V( ...
  tfm.qr_abs_target_V, target_position, target_heading, level);

if debug
  % Lower level plots showing transformed states position-velocity space
  base_pos = rotate2D(tfm.aas{1}.getPosition - target_position', ...
    -target_heading);
  base_vel = rotate2D(tfm.aas{1}.getVelocity, -target_heading);
  
  %% x direction
  subplot(1,3,2)
  plot(base_pos(1), base_vel(1), 'k.')
  hold on
  g2D = proj2D(tfm.qr_abs_target_V.g, [], [0 0 1 1]);
  data3D = min(tfm.qr_abs_target_V.data, [], 3);
  data2D = min(data3D, [], 4);
  contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.2:15)
  axis square
  
  % Manual switching curves
  x0left = -1.5*tfm.qr_abs_target_V.g.dx(1);
  y0left = 10;
  
  x0right = 0; %tfm.qr_abs_target_V.g.dx(1);
  y0right = 10 - 1.5*tfm.qr_abs_target_V.g.dx(2);
  
  % Plot switching curves
  tt = linspace(-8, 1, 50);
  x = 0.5 * 3 * tt.^2 + y0left*tt + x0left;
  y = 3*tt + y0left;
  plot(x, y, 'k-.')
  x = 0.5 * 3 * tt.^2 + y0right*tt + x0right;
  y = 3*tt + y0right;
  plot(x, y, 'k-.')
  
  %% y direction
  subplot(1,3,3)
  plot(base_pos(2), base_vel(2), 'k.')
  hold on
  g2D = proj2D(tfm.qr_abs_target_V.g, [], [1 1 0 0]);
  data3D = min(tfm.qr_abs_target_V.data, [], 1);
  data2D = squeeze(min(data3D, [], 2));
  contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.2:15)
  axis square
end
drawnow;

%% Integration
tMax = 20;
t = 0:tfm.dt:tMax;
for i = 1:length(t)
  % Just get to pose...
  u = tfm.aas{1}.getToPose(tfm, target_position, target_heading, debug);
  
  % Main plot
  if debug
    subplot(1,3,1)
  end
  tfm.aas{1}.updateState(u, tfm.dt);
  tfm.aas{1}.plotPosition;
  tfm.aas{1}.plot_abs_target_V( ...
    tfm.qr_abs_target_V, target_position, target_heading, level);
  
  % Lower level plots
  if debug
    base_pos = rotate2D(tfm.aas{1}.getPosition - target_position', ...
      -target_heading);
    base_vel = rotate2D(tfm.aas{1}.getVelocity, -target_heading);
    
    subplot(1,3,2)
    plot(base_pos(1), base_vel(1), 'k.')
    
    subplot(1,3,3)
    plot(base_pos(2), base_vel(2), 'k.')
  end
  drawnow;
  
  if isempty(u)
    break;
  end
end

end