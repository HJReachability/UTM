function uQuad2uPl4_test()
% uQuad2uPl4_test()
%
% Tests the function that converts controls (acceleration in x and y 
% directions) in a Quadrotor to controls in a Plane4 (acceleration and 
% turn rate)
%
% Mo Chen, 2016-02-01

% Initialize Plane4
pl4 = Plane4([0 0 0 1]);
figure;
pl4.plotPosition;
drawnow

xlim([-5 15])
ylim([-5 15])

% Example Quadrotor x- and y- acceleration
uQuad = [1 1];

% Simulate
tMax = 5;
dt = 0.1;
t = 0:dt:tMax;

for i = 1:length(t)
  uPl4 = pl4.uQuad2uPl4(uQuad);
  pl4.updateState(uPl4, dt);
  pl4.plotPosition;
  drawnow
end
end