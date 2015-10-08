clear all
close all

% Specify N random points uniformly distributed in [0 1]^2
N = 100;
points = rand(2,N);

% Plot the random points
figure;
plot(points(1,:), points(2,:), '.'); hold on

% Create a random point this_pt
this_pt = rand(2,1);
plot(this_pt(1), this_pt(2), 'x')

% Plot a random fixed distance from the this_pt
threshold = rand;
theta = linspace(0, 2*pi, 100);
plot(this_pt(1) + threshold*cos(theta), ...
  this_pt(2) + threshold*sin(theta), ':')

% Compute and plot nearby points
near_pts = nearby_pts(this_pt, points, threshold);
plot(near_pts(1,:), near_pts(2,:), 'o')

axis equal

