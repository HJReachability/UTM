function RS_rotation_abs_target_test()
% function RS_rotation_abs_target_test()
%
% Tests for using a base create platoon reachable set to synthesize a
% controller to move to the origin. 
%
% The base reachable set assumes that the
% target velocity is orig_v = [3 0]. However, the true target set as a
% target velocity that is rotated by some random angle.
%
% In order to use the base reachable set to synthesize a proper controller,
% states relative to the target state must be rotated by negative of the
% random angle
%
% Mo Chen, 2015-10-26

addpath('..')

% Compute and visualize reachable sets
orig_v = [3 0];
[base_TTR_out, rot_TTR_out, theta] = RS_rotate_visualize(orig_v);

% Target position
pt = [0 0];
vt = rotate2D(orig_v, theta);
xt = [pt(1) vt(1) pt(2) vt(2)];
disp(['Target state: ' num2str(xt)])

% Initial position and velocity
orig_p = [-15 5];
rot_p = rotate2D(orig_p, theta);
v = rotate2D(orig_v, theta);
x = [rot_p(1) v(1) rot_p(2) v(2)];
q1 = Quadrotor(x);
q2 = Quadrotor(x);

% Initialize plot
figure;
q1.plotPosition;
q2.plotPosition;
title('t = 0')
xlim([-15 15])
grid on
axis equal
drawnow;

% Simulate using both the rotated reachable set directly and using the base
% reachable set with rotated states
simulate_rotRS(q1, rot_TTR_out);
simulate_baseRS(q2, base_TTR_out, theta);
end

function simulate_baseRS(q, base_TTR_out, theta)
% function simulate_baseRS(q, base_TTR_out, theta)
%
% Simulates the case where the base reachable set is used, and the actual
% vehicle states are rotated appropriately to the frame of the base
% reachable set
%
% Mo Chen, 2015-10-26

% Time vector
dt = 0.1;
tMax = 10;
t = 0:dt:tMax;
uMin = -1.7;
uMax = 1.7;

% State in the frame of the base reachable set (vehicle frame)
base_pos = rotate2D(q.getPosition, -theta);
base_v = rotate2D(q.getVelocity, -theta);
base_x = [base_pos(1) base_v(1) base_pos(2) base_v(2)];
for i = 1:length(t)
  % Gradient in the frame of base reachable set
  base_p = calculateCostate(base_TTR_out.g, base_TTR_out.grad, base_x);

  ux = (base_p(2)>=0)*uMin + (base_p(2)<0)*uMax;
  uy = (base_p(4)>=0)*uMin + (base_p(4)<0)*uMax;
  u = [ux; uy];
  
  % Apply control in the global frame
  q.updateState(rotate2D(u, theta), dt);
  
  % Update and plot position
  q.plotPosition;
  title(['t = ' num2str(t(i))])
  drawnow
  
  % Check value function using state in the frame of base reachable set
  base_pos = rotate2D(q.getPosition, -theta);
  base_v = rotate2D(q.getVelocity, -theta);
  base_x = [base_pos(1) base_v(1) base_pos(2) base_v(2)];  
  valuex = eval_u(base_TTR_out.g, base_TTR_out.value, base_x);
  
  % Exit when target is within 0.25 seconds
  if valuex < 0.25
    break;
  end
end
end

function simulate_rotRS(q, rot_TTR_out)
% function simulate_rotRS(q, rot_TTR_out)
%
% Simulates the case where a reachable set of the specific velocity is used
% to drive a vehicle to the target set
%
% Mo Chen, 2015-10-26

% Time vector
dt = 0.1;
tMax = 10;
t = 0:dt:tMax;
uMin = -1.7;
uMax = 1.7;
for i = 1:length(t)
  % Gradient
  rot_p = calculateCostate(rot_TTR_out.g, rot_TTR_out.grad, q.x);

  ux = (rot_p(2)>=0)*uMin + (rot_p(2)<0)*uMax;
  uy = (rot_p(4)>=0)*uMin + (rot_p(4)<0)*uMax;
  u = [ux; uy];
  
  % Update and plot state
  q.updateState(u, dt);
  
  q.plotPosition;
  title(['t = ' num2str(t(i))])
  drawnow
  
  % Exit if ETA 0.25s
  valuex = eval_u(rot_TTR_out.g, rot_TTR_out.value, q.x);
  if valuex < 0.25
    break;
  end
end
end

function [base_TTR_out, rot_TTR_out, theta] = RS_rotate_visualize(v)
% function [base_TTR_out, rot_TTR_out, theta] = RS_rotate_visualize(v)
%
% Computes and plots the base reachable set with a generic velocity target,
% and the specific (rotated) reachable set with a specific velocity target.
%
% Mo Chen, 2015-10-26

%% Create the base reachable set and reconstruct
disp(['Original velocity = ' num2str(v)])
x = [0 v(1) 0 v(2)];
[grids, datas, tau] = quad_abs_target_2D(x, 0);

% Reconstruct the base reachable set
gridLim = ...
  [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
[~, ~, base_TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

% Visualize the base reachable set
[g2D, data2D] = proj2D(base_TTR_out.g, [0 1 0 1], ...
  base_TTR_out.g.N([1 3]), base_TTR_out.value, v);
figure
contour(g2D.xs{1}, g2D.xs{2}, data2D, [4 4])
hold on

%% Rotate the velocity vector
theta = 2*pi*rand;
disp(['Rotating by ' num2str(theta*180/pi) ' degrees.'])
v = rotate2D(v, theta);
disp(['New velocity = ' num2str(v)])

x([2 4]) = v;
[grids, datas, tau] = quad_abs_target_2D(x, 0);

% Reconstruct the rotated reachable set
gridLim = ...
  [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
[~, ~, rot_TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

% Visualize the rotated reachable set
[g2D, data2D] = proj2D(rot_TTR_out.g, [0 1 0 1], ...
  rot_TTR_out.g.N([1 3]), rot_TTR_out.value, v);
contour(g2D.xs{1}, g2D.xs{2}, data2D, [4 4])

grid on
end