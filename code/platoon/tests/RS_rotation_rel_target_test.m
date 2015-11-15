function RS_rotation_rel_target_test()
% function RS_rotation_rel_target_test()
%
% Tests for using a base create platoon reachable set to synthesize a
% controller to move to a relative target behind a quadrotor. 
%
% The base reachable set assumes that the target state is orig_p = [-3 0],
% orig_v = [0 0], and the actual velocity of the other vehicle is [3 0]. In
% reality, the other vehicle has the same speed but in another direction,
% and the target relative position rot_p is a rotation of orig_p by the
% same angle.
%
% In order to use the base reachable set to synthesize a proper controller,
% states relative to the target state must be rotated by negative of the
% random angle
%
% Mo Chen, 2015-10-27

addpath('..')

% Compute and visualize reachable sets
base_TTR_out = RS_visualize;

% Random rotation
theta = 2*pi*rand;
disp(['Angle: ' num2str(theta*180/pi) ' degrees'])
figure;
plot(20*[0 cos(theta)], 20*[0 sin(theta)], 'k--')
hold on

% Leader velocity
orig_v2 = [3 0];
v2 = rotate2D(orig_v2, theta);
q2 = Quadrotor([0 v2(1) 0 v2(2)]);

% Target relative position
rel_p = [-5 0];
rel_pt = rotate2D(rel_p, theta);
disp(['Target relative position: ' num2str(rel_pt)])

% Joiner initial position
offset = [-1 -1];
orig_p = rel_p + offset;
rot_p = rotate2D(orig_p, theta);
q1 = Quadrotor([rot_p(1) 0 rot_p(2) 0]);

% Initialize plot
q1.plotPosition('r');
q2.plotPosition('b');
title('t = 0')
grid on
axis equal
drawnow;

% Simulate using the base reachable set with rotated states
simulate_baseRS(q1, q2, base_TTR_out, rel_pt, theta);
end

function simulate_baseRS(q1, q2, base_TTR_out, rel_pt, theta)
% function simulate_baseRS(q, base_TTR_out, theta)
%
% Simulates the case where the base reachable set is used, and the actual
% vehicle states are rotated appropriately to the frame of the base
% reachable set
%
% q2 is leader, q1 tries to join platoon
%
% Mo Chen, 2015-10-27

% Time vector
dt = 0.1;
tMax = 5;
t = 0:dt:tMax;
uMin = -1.7;
uMax = 1.7;

% State in the frame of the base reachable set (vehicle frame)
base_pos = rotate2D(q1.getPosition - q2.getPosition - rel_pt', -theta);
base_v = rotate2D(q1.getVelocity - q2.getVelocity, -theta);
base_x = [base_pos(1) base_v(1) base_pos(2) base_v(2)];
valuex = eval_u(base_TTR_out.g, base_TTR_out.value, base_x);
disp(valuex)

if valuex > 100 || isnan(valuex)
  disp(base_x)
  error('Value too high to start!')
end

for i = 1:length(t)
  % Exit when target is within 0.25 seconds
  if valuex < 0.25
    break;
  end
  
  % Gradient in the frame of base reachable set
  base_p = calculateCostate(base_TTR_out.g, base_TTR_out.grad, base_x);

  ux = (base_p(2)>=0)*uMin + (base_p(2)<0)*uMax;
  uy = (base_p(4)>=0)*uMin + (base_p(4)<0)*uMax;
  u = [ux; uy];
  
  % Apply control in the global frame
  q2.updateState([0 0], dt);
  q1.updateState(rotate2D(u, theta), dt);
  
  % Update and plot position
  q1.plotPosition;
  q2.plotPosition;
  title(['t = ' num2str(t(i))])
  drawnow
  
  % Check value function using state in the frame of base reachable set
  base_pos = rotate2D(q1.getPosition - q2.getPosition - rel_pt', -theta);
  base_v = rotate2D(q1.getVelocity - q2.getVelocity, -theta);
  base_x = [base_pos(1) base_v(1) base_pos(2) base_v(2)];  
  valuex = eval_u(base_TTR_out.g, base_TTR_out.value, base_x);
  disp(valuex)
end
end

function base_TTR_out = RS_visualize()
% function [base_TTR_out, rot_TTR_out, theta] = RS_rotate_visualize(v)
%
% Computes and plots the base reachable set with a generic velocity target,
% and the specific (rotated) reachable set with a specific velocity target.
%
% Mo Chen, 2015-10-27

%% Create the base reachable set and reconstruct
x = [0 0 0 0];
[grids, datas, tau] = quad_rel_target_2D(x, 0);

% Reconstruct the base reachable set
gridLim = ...
  [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
[~, ~, base_TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

% Visualize the base reachable set
[g2D, data2D] = proj2D(base_TTR_out.g, base_TTR_out.value, [0 1 0 1], ...
    [0 0]);
figure
contour(g2D.xs{1}, g2D.xs{2}, data2D, [4 4])
hold on
end