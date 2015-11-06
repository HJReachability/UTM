function followPath_test()
% Tests Quadrotor.followPath
addpath('..')

LQR = 1;

% Initialize quadrotor
q = Quadrotor([0 0 0 0]);
figure;
q.plotPosition;
hold on

% Plot bounds
xlim([-50 50])
ylim([-50 50])
axis square

% Random path
target_origin = 5*rand(2,1);
target_dir = 100*rotate2D([1 0], 2*pi*rand);
speed = 2+3*rand;
ptf = Linpath(target_origin, target_dir, speed); % path to follow
ptf.lpPlot;

title(['Speed = ' num2str(speed)])
drawnow;

% Integration parameters
dt = 0.1;
tMax = 10;
t = 0:0.1:tMax;

% Integration
for i = 1:length(t)
  u = q.followPath(ptf, ptf.speed, LQR);
  disp(['Target speed: ' num2str(speed), '; speed: ' num2str(q.getSpeed)])
  q.updateState(u, dt);
  q.plotPosition;
  drawnow;
end
end