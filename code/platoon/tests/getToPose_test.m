function getToPose_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('qr_create_platoon_V');
debug = 1;

% Random heading
pos_theta = 2*pi*rand;
vel_theta = pos_theta;% - pi/2 + pi*rand;

% Add quadrotors to TFM
init_x = [0 10*cos(pos_theta) 0 10*sin(pos_theta)];
tfm.addActiveAgents(Quadrotor(init_x));

% Target state (random angle at 50 distance away; angle may be different
% from heading)
target_dist = 50;
target_position = rotate2D([target_dist 0], pos_theta);
figure;
quiver(target_position(1), target_position(2), ...
  10*cos(pos_theta), 10*sin(pos_theta), 'rx')
hold on

% Plot initial setup
level = 8;
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition('b');
  tfm.aas{j}.plot_abs_target_V(tfm.qr_create_platoon_V, ...
    level, target_position, vel_theta);
end
xlim(1.5*target_dist*[-1 1])
ylim(1.5*target_dist*[-1 1])
axis square
drawnow;

% Integration parameters
tMax = 10;
t = 0:tfm.dt:tMax;

% Integrate
for i = 1:length(t)
  for j = 1:length(tfm.aas)
    u = tfm.aas{j}.getToPose(tfm, target_position, vel_theta, debug);
    
    tfm.aas{j}.updateState(u, tfm.dt);
    tfm.aas{j}.plotPosition;
    
    tfm.aas{j}.plot_abs_target_V(tfm.qr_create_platoon_V, ...
      level, target_position, vel_theta);
  end
  drawnow;
end


end