function simulateChangeHighway(save_figures)
% simulateFormPlatoon()
%
% Simulates 5 quadrotors forming a single platoon on a highway

addpath('..')

if nargin < 1
  save_figures = false;
end

%% TFM
tfm = TFM;
tfm.computeRS('qr_rel_target_V');
tfm.computeRS('qr_abs_target_V');
tfm.computeRS('qr_qr_safe_V');

%% Highways
theta1 = 2*pi*rand;
theta2 = theta1 + 45*pi/180;
hw_length = 200;
z0b = hw_length*[-1 0];
z1b = hw_length*[1 0];

% Highway 1
z0 = rotate2D(z0b, theta1);
z1 = rotate2D(z1b, theta1);
hw1 = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw1);

% Highway 2
z0 = rotate2D(z0b, theta2);
z1 = rotate2D(z1b, theta2);
hw2 = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw2);

% plot
figure;
hw1.lpPlot;
hold on
hw2.lpPlot;
hold on

%% Quadrotors
% Platoon 1 (4 vehicles)
x0 = -10*tfm.ipsd;
xs = x0 + 3*tfm.ipsd: -tfm.ipsd: x0;
ys = zeros(size(xs));
xs_ys = rotate2D([xs; ys], theta1);
xs = xs_ys(1,:);
ys = xs_ys(2,:);
for j = 1:length(xs)
  q = Quadrotor([xs(j) 0 ys(j) 0]);
  tfm.regVehicle(q);
  if j == 1
    p1 = Platoon(q, hw1, tfm);
  else
    p1.insertVehicle(q, j);
  end
end

% Platoon 2 (3 vehicles)
x0 = -5*tfm.ipsd;
xs = x0 + 2*tfm.ipsd: -tfm.ipsd: x0;
ys = zeros(size(xs));
xs_ys = rotate2D([xs; ys], theta2);
xs = xs_ys(1,:);
ys = xs_ys(2,:);
for j = 1:length(xs)
  q = Quadrotor([xs(j) 0 ys(j) 0]);
  tfm.regVehicle(q);
  if j == 1
    p2 = Platoon(q, hw2, tfm);
  else
    p2.insertVehicle(q, j);
  end
end

for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
end
title('t=0')
axis square
drawnow

if save_figures
  fig_dir = [fileparts(mfilename('fullpath')) '/' mfilename '_figs'];
  if ~exist(fig_dir, 'dir')
    cmd = ['mkdir -p ' fig_dir];
    system(cmd)
  end
  export_fig([fig_dir '/0'], '-png', '-m2')
end


%% Integration
tMax = 23;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;
  
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = controlLogic(tfm, tfm.aas{j}, p1, p2);

    else
      u{j} = uSafe{j};
    end
    
    tfm.aas{j}.updateState(u{j}, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  title(['t=' num2str(t(i))])
  drawnow

  if save_figures
    export_fig([fig_dir '/' num2str(i)], '-png', '-m2')
  end
end

tfm.printBreadthFirst;
end

function u = controlLogic(tfm, veh, p1, p2)

if ~isempty(veh.p) && (veh.ID == 1 || veh.ID == 3) && veh.p == p1
  [~, dist] = tfm.hws{2}.highwayPos(veh.getPosition);
  
  if dist < 50
    u = veh.joinPlatoon(p2, tfm);
    return
  end
end

if strcmp(veh.q, 'Free')
  u = veh.joinPlatoon(p2, tfm);
  return
end

if strcmp(veh.q, 'Leader')
  u = veh.followPath(veh.p.hw);
  return
end

% If already in the platoon, simply follow it
u = veh.followPlatoon(tfm);
end