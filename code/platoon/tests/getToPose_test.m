function getToPose_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('qr_abs_target_V');

N = 10;
for i = 1:N
  getToPoseSingle(tfm);
end


end

function getToPoseSingle(tfm)

debug = 1;

% Random heading

pos_theta = 0;
vel_theta = 0;
%pos_theta = 2*pi*rand;
%vel_theta = pos_theta - pi/2 + pi*rand;

% Add quadrotors to TFM
init_x = [0 10 -50 0];
tfm.aas = {};
tfm.addActiveAgents(Quadrotor(init_x));

% Target state (random angle at 50 distance away; angle may be different
% from heading)
target_dist = 50;
target_position = rotate2D([target_dist 0], pos_theta);
figure;
quiver(target_position(1), target_position(2), ...
  10*cos(vel_theta), 10*sin(vel_theta), 'rx')
hold on

% Plot initial setup
level = tfm.rtt;
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition('b');
  tfm.aas{j}.plot_abs_target_V( ...
    tfm.qr_abs_target_V, level, target_position, vel_theta);
end
xlim(1.2*target_dist*[-1 1])
ylim(1.2*target_dist*[-1 1])
axis square
drawnow;

% Integration parameters
tMax = 15;
t = 0:tfm.dt:tMax;

% Integrate
for i = 1:length(t)
  if i == length(t)
    p = tfm.aas{1}.getPosition - target_position';
    figure;
    [g2D, data2D] = proj2D(tfm.qr_abs_target_V.g, [1 0 1 0], ...
      tfm.qr_abs_target_V.g.N([2 4]), tfm.qr_abs_target_V.value, p);
    
    contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.2:13)
    hold on
    v = tfm.aas{1}.getVelocity;
    plot(v(1), v(2), '*')
    title(['p = ' num2str(p')])
    
    figure;
    [g2D, data2D] = proj2D(tfm.qr_abs_target_V.g, [1 0 1 0], ...
      tfm.qr_abs_target_V.g.N([2 4]), tfm.qr_abs_target_V.grad{2}, p);
    
    contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.2:13)
    hold on
    v = tfm.aas{1}.getVelocity;
    plot(v(1), v(2), '*')
    title(['p = ' num2str(p')])
    
    base_p = calculateCostate(tfm.qr_abs_target_V.g, tfm.qr_abs_target_V.grad, [p(1) v(1) p(2) v(2)])
  end
  
  for j = 1:length(tfm.aas)
    u = tfm.aas{j}.getToPose(tfm, target_position, vel_theta, debug);
    
    tfm.aas{j}.updateState(u, tfm.dt);
    tfm.aas{j}.plotPosition;
    
    tfm.aas{j}.plot_abs_target_V( ...
      tfm.qr_abs_target_V, level, target_position, vel_theta);
  end
  drawnow;
end

% Project 4D reachable set to position space
p = tfm.aas{1}.getPosition - target_position';
v = tfm.aas{1}.getVelocity;
figure;
[g2D, data2D] = proj2D(tfm.qr_abs_target_V.g, [1 0 1 0], ...
  tfm.qr_abs_target_V.g.N([2 4]), tfm.qr_abs_target_V.value, p);

contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.2:13)
hold on

plot(v(1), v(2), '*')
title(['p = ' num2str(p')])

figure;
[g2D, data2D] = proj2D(tfm.qr_abs_target_V.g, [1 0 1 0], ...
  tfm.qr_abs_target_V.g.N([2 4]), tfm.qr_abs_target_V.grad{2}, p);

contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:0.2:13)
hold on
v = tfm.aas{1}.getVelocity;
plot(v(1), v(2), '*')
title(['p = ' num2str(p')])

base_p = calculateCostate(tfm.qr_abs_target_V.g, tfm.qr_abs_target_V.grad, [p(1) v(1) p(2) v(2)])
keyboard
end