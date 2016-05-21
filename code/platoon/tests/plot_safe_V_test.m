function plot_safe_V_test()

addpath(genpath('..'));

speed = 5;
cr = 2; % collision radius
level = 0;

filename = ['saved/' mfilename '_' num2str(speed) '_' num2str(cr) '.mat'];
if exist(filename, 'file')
  load(filename)
else
  [safe_V.g, safe_V.data] = pl_pl_collision_3D(cr, speed);
  save(filename, 'safe_V');
end

pl1 = Plane([0 0 0]);
pl1.speed = speed;

pl2 = Plane([10 10 pi]);
pl2.speed = speed;

figure;
pl1.plotPosition();
pl2.plotPosition();
pl1.plot_safe_V(pl2, safe_V, level);
pl2.plot_safe_V(pl1, safe_V, level);
xlim([-15 25])
ylim([-20 20])
axis square
drawnow

u1 = -pl1.wMax;
u2 = 0.9*pl2.wMax;

dt = 0.1;
tMax = 100;
t = 0:dt:tMax;

for i = 1:length(t)
  pl1.updateState(u1, dt);
  pl2.updateState(u2, dt);

  pl1.plotPosition();
  pl2.plotPosition();
  pl1.plot_safe_V(pl2, safe_V, level);
  pl2.plot_safe_V(pl1, safe_V, level);
  drawnow
end