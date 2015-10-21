clear all;
close all;
addpath('..')
%% Create highway (also calls quad2D_liveness to compute reachable set)
v = 3;
z0 = [-30 -15];
z1 = [80 40];
hw = highway(z0, z1, v);

%% Plot 2D slices of reachable sets
N = 4;
spC = ceil(sqrt(N));
spR = ceil(N/spC);

for i = 1:N
  subplot(spR, spC, i)
  
  % Choose a random velocity
  velocity = 1.5*(-v + 2*v*rand(2,1));
  
  % Project to 2D
  [g2D, value2D] = proj2D(hw.liveV.g, [0 1 0 1], hw.liveV.g.N([1 3]), ...
    hw.liveV.data, velocity);
  
  % Contour plot
  contour(g2D.xs{1}, g2D.xs{2}, value2D, 0:0.5:10)
  title(['velocity = [' num2str(velocity(1)) ' ' num2str(velocity(2)) ']'])
  grid on
  axis equal
end