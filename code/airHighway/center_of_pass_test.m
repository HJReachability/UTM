clear all;
close all;

% Specify N random points uniformly distributed in [0 1]^2
N = 100;
points = rand(2,N);

% Plot the random points
figure;
for i = 1:size(points,2)
  plot(points(1,i), points(2,i), '.'); hold on
end

% Compute and plot center of mass (should be near (0.5, 0.5)
com = center_of_mass(points);
plot(com(1), com(2), 'x')