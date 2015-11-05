function RS_rotation_safety_test()
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

% Random rotation
theta = 2*pi*rand;
disp(['Angle: ' num2str(theta*180/pi) ' degrees'])

% Random speed
v = [4*rand 0];
rot_v = rotate2D(v, theta);
disp(['Speed: ' num2str(v(1)) '; velocity: ' num2str(rot_v)])

% Compute and visualize reachable sets
base_TTR_out = RS_visualize(rot_v);

N = 5;
for i = 1:N
% Random rotation
theta = 2*pi*rand;
disp(['Angle: ' num2str(theta*180/pi) ' degrees'])

figure;
plot(20*cos(theta)*[-1 1], 20*sin(theta)*[-1 1], 'k--')
hold on

% Evader initial position
pe = [-10 0];
ve = [4 0];
rot_pe = rotate2D(pe, theta);
rot_ve = rotate2D(ve, theta);
qe1 = quadrotor(1, [rot_pe(1) rot_ve(1) rot_pe(2) rot_ve(2)]);
qe2 = quadrotor(1, [rot_pe(1) rot_ve(1) rot_pe(2) rot_ve(2)]);

% Pursuer state
pp = [10 1];
vp = -[4 0];
rot_pp = rotate2D(pp, theta);
rot_vp = rotate2D(vp, theta);
qp = quadrotor(2, [rot_pp(1) rot_vp(1) rot_pp(2) rot_vp(2)]);

% Initialize plot
qe1.plotPosition('r');
qe2.plotPosition('b');
qp.plotPosition('k');
title('t = 0')
grid on
axis equal
drawnow;

% Simulate using the base reachable set with rotated states
simulate_baseRS(qe1, qe2, qp, base_TTR_out, theta);
end
end

function simulate_baseRS(qe1, qe2, qp, base_TTR_out, theta)
% function simulate_baseRS(q, base_TTR_out, theta)
%
% Simulates the case where the base reachable set is used, and the actual
% vehicle states are rotated appropriately to the frame of the base
% reachable set
%
% q1 is leader, q2 tries to join platoon
%
% Mo Chen, 2015-10-27

% Time vector
dt = 0.1;
tMax = 6;
t = 0:dt:tMax;
uMin = -1.7;
uMax = 1.7;

% State in the frame of the base reachable set (vehicle frame)
base_pos1 = rotate2D(qp.getPosition - qe1.getPosition, -theta);
base_v1 = rotate2D(qp.getVelocity - qe1.getVelocity, -theta);
base_x1 = [base_pos1(1) base_v1(1) base_pos1(2) base_v1(2)];
valuex1 = eval_u(base_TTR_out.g, base_TTR_out.value, base_x1);

% State in absolute frame
base_pos2 = qp.getPosition - qe2.getPosition;
base_v2 = qp.getVelocity - qe2.getVelocity;
base_x2 = [base_pos2(1) base_v2(1) base_pos2(2) base_v2(2)];
valuex2 = eval_u(base_TTR_out.g, base_TTR_out.value, base_x2);


for i = 1:length(t)
  if valuex1 <= 2
    % Gradient in the frame of base reachable set
    base_p1 = calculateCostate(base_TTR_out.g, base_TTR_out.grad, base_x1);
    ux1 = (base_p1(2)>=0)*uMin + (base_p1(2)<0)*uMax;
    uy1 = (base_p1(4)>=0)*uMin + (base_p1(4)<0)*uMax;
    u1 = [ux1; uy1];
    u1 = rotate2D(u1, theta);
  else
    u1 = [0; 0];
  end
  
  if valuex2 <= 2
    % Gradient in the absolute frame
    base_p2 = calculateCostate(base_TTR_out.g, base_TTR_out.grad, base_x2);
    ux2 = (base_p2(2)>=0)*uMin + (base_p2(2)<0)*uMax;
    uy2 = (base_p2(4)>=0)*uMin + (base_p2(4)<0)*uMax;
    u2 = [ux2; uy2];
  else
    u2 = [0; 0];
  end  
  
  % Apply control in the global frame
  qp.updateState([0 0]);
  qe1.updateState(u1);
  qe2.updateState(u2);
  
  % Update and plot position
  qe1.plotPosition;
  qe2.plotPosition;
  qp.plotPosition;
  title(['t = ' num2str(t(i))])
  drawnow
  
  % Check value function using state in the frame of base reachable set
  base_pos1 = rotate2D(qp.getPosition - qe1.getPosition, -theta);
  base_v1 = rotate2D(qp.getVelocity - qe1.getVelocity, -theta);
  base_x1 = [base_pos1(1) base_v1(1) base_pos1(2) base_v1(2)];  
  valuex1 = eval_u(base_TTR_out.g, base_TTR_out.value, base_x1);
  
  % State in absolute frame
  base_pos2 = qp.getPosition - qe2.getPosition;
  base_v2 = qp.getVelocity - qe2.getVelocity;
  base_x2 = [base_pos2(1) base_v2(1) base_pos2(2) base_v2(2)];
  valuex2 = eval_u(base_TTR_out.g, base_TTR_out.value, base_x2);
end
end

function base_TTR_out = RS_visualize(v)
% function [base_TTR_out, rot_TTR_out, theta] = RS_rotate_visualize(v)
%
% Computes and plots the base reachable set with a generic velocity target,
% and the specific (rotated) reachable set with a specific velocity target.
%
% Mo Chen, 2015-10-27

%% Create the base reachable set and reconstruct
d = 2; % Collision "radius"
visualize = 0;
[data, g, tau] = quad_quad_collision_2D(d, visualize);

% Reconstruct the base reachable set
gridLim = [g.min-1 g.max+1; g.min-1 g.max+1];
[~, ~, base_TTR_out] = ...
  recon2x2D(tau, {g; g}, {data; data}, gridLim, tau(end));

% Visualize the base reachable set
[g2D, data2D] = proj2D(base_TTR_out.g, [0 1 0 1], ...
  base_TTR_out.g.N([1 3]), base_TTR_out.value, v);
figure
contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.5:2.5)
hold on
end