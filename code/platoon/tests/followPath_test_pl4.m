function followPath_test_pl4()
% Tests Plane4.followPath
addpath('..')

LQR = 1;

% Initialize Plane4
pl4 = Plane4([0 0 0 7]);
figure;
pl4.plotPosition;
hold on

% Plot bounds
xlim([-50 50])
ylim([-50 50])
axis square

% Random path
theta = 2*pi*rand;
target_origin = 5*rand(2,1);
target_dest = 100*rotate2D([1 0], theta);
speed = 7;
ptf = Linpath(target_origin, target_dest, speed); % path to follow
ptf.lpPlot;

title(['Speed = ' num2str(speed)])
drawnow;

% Integration parameters
dt = 0.1;
tMax = 10;
t = 0:0.1:tMax;

% Integration
for i = 1:length(t)
  u = pl4.followPath(ptf, ptf.speed, LQR);

  disp(['Target speed: ' num2str(speed), ...
    '; speed: ' num2str(norm(pl4.getVelocity))]);
  pl4.updateState(u, dt);
  pl4.plotPosition;
  drawnow;
end

end