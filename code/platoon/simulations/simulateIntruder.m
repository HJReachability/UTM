function simulateIntruder(save_figures, fig_formats)
% simulateFormPlatoon()
%
% Simulates 5 quadrotors forming a single platoon on a highway

addpath(genpath('..'))

if nargin < 1
  save_figures = false;
end

if nargin < 2
  fig_formats = {'png'};
end

%% TFM
tfm = TFM;
tfm.computeRS('qr_rel_target_V');
tfm.computeRS('qr_abs_target_V');
tfm.computeRS('qr_qr_safe_V');
tfm.ipsd = 10;
%% Highways
theta = 2*pi*rand;
hw_length = 300;
z0 = hw_length*[-0.1 0];
z1 = hw_length*[1 0];

% Highway 1
z0 = rotate2D(z0, theta);
z1 = rotate2D(z1, theta);
hw = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw);

% plot
f = figure;
hw.lpPlot;
hold on

f.Children.FontSize = 16;
f.Position(1:2) = [200 200];
f.Position(3:4) = [1000 750];

%% Quadrotors
% Platoon 1 (4 vehicles)
x0 = 0;
xs = x0: -tfm.ipsd: x0 - 3*tfm.ipsd;
ys = zeros(size(xs));
xs_ys = rotate2D([xs; ys], theta);
xs = xs_ys(1,:);
ys = xs_ys(2,:);
vq = [10 0];
vq = rotate2D(vq, theta);
for j = 1:length(xs)
  q = Quadrotor([xs(j) vq(1) ys(j) vq(2)]);
  tfm.regVehicle(q);
  if j == 1
    p = Platoon(q, hw, tfm);
  else
    p.insertVehicle(q, j);
  end
end

% Intruder
pin = [250 50];
vin = [10 0];
vin = rotate2D(vin, 9*pi/8);
pin = rotate2D(pin, theta);
vin = rotate2D(vin, theta);
intruder = Quadrotor([pin(1) vin(1) pin(2) vin(2)]);
tfm.regVehicle(intruder);

for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
end


title('t=0')
axis equal
drawnow

if save_figures
  fig_dir = [fileparts(mfilename('fullpath')) '\' mfilename '_figs'];
  if ~exist(fig_dir, 'dir')
    cmd = ['mkdir ' fig_dir];
    system(cmd)
  end
  
  for ii = 1:length(fig_formats)
    if strcmp(fig_formats{ii}, 'png')
      export_fig([fig_dir '/0'], '-png', '-m2', '-transparent')
    end
    
    if strcmp(fig_formats{ii}, 'pdf')
      export_fig([fig_dir '/0'], '-pdf', '-m2', '-transparent')
    end
    
    if strcmp(fig_formats{ii}, 'fig')
      savefig([fig_dir '/0.fig']);
    end
  end
end


%% Integration
tMax = 25;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;

  safe(length(tfm.aas)) = 1;
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = controlLogic(tfm, tfm.aas{j});

    else
      
      u{j} = uSafe{j};
    end
    
    tfm.aas{j}.updateState(u{j}, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  title(['t=' num2str(t(i))])
  drawnow

  if save_figures
    for ii = 1:length(fig_formats)
      if strcmp(fig_formats{ii}, 'png')
        export_fig([fig_dir '/' num2str(i)], '-png', '-m2', '-transparent')
      end

      if strcmp(fig_formats{ii}, 'pdf')
        export_fig([fig_dir '/' num2str(i)], '-pdf', '-m2', '-transparent')
      end

      if strcmp(fig_formats{ii}, 'fig')
        savefig([fig_dir '/' num2str(i) '.fig']);
      end
    end
  end
end

tfm.printBreadthFirst;
end

function u = controlLogic(tfm, veh)
if strcmp(veh.q, 'Free')
  u = [0; 0];
  return
end

if strcmp(veh.q, 'Leader')
  u = veh.followPath(veh.p.hw);
  return
end

% If already in the platoon, simply follow it
u = veh.followPlatoon(tfm);
end