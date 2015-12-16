function simulateFormPlatoon(save_figures)
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

%% Highway
theta = 2*pi*rand;
hw_length = 350;
z0 = [0 0];
z1 = hw_length*[1 0];
z1 = rotate2D(z1, theta);
hw = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw);

% plot
f = figure;

hw.lpPlot;
hold on
f.Children.FontSize = 16;

%% Quadrotors
xs = 100:-25:0;
ys = sign(rand(size(xs))-0.5).*(125 - xs);
xs_ys = rotate2D([xs; ys], theta);
xs = xs_ys(1,:);
ys = xs_ys(2,:);
for j = 1:length(xs)
  tfm.regVehicle(Quadrotor([xs(j) 0 ys(j) 0]));
  tfm.aas{j}.plotPosition;
end

title('t=0', 'FontSize', 16)
drawnow

if save_figures
  fig_dir = [fileparts(mfilename('fullpath')) '/' mfilename '_figs'];
  if ~exist(fig_dir, 'dir')
    cmd = ['mkdir -p ' fig_dir];
    system(cmd)
  end
  export_fig([fig_dir '/0'], '-png', '-m2')
end

%% Target entry point
tep = rotate2D([150 0], theta);

%% Integration
tMax = 23;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;
  
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = controlLogic(tfm, tfm.aas{j}, tep);

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

function u = controlLogic(tfm, veh, tep)
%% If no platoon has been formed, then try to go on the highway to form one
% Also, plot the reachable set for getting onto the highway for the first
% vehicle
if isempty(tfm.hws{1}.ps)
  u = veh.goOnHighway(tfm.hws{1}, tep, tfm);
  tfm.aas{1}.plot_abs_target_V(tfm.qr_abs_target_V, tep, ...
  tfm.hws{1}.getHeading, tfm.rtt);
  tfm.aas{1}.plot_safe_V(tfm.aas{2}, tfm.qr_qr_safe_V, tfm.safetyTime)
  return
else
  tfm.aas{1}.unplot_abs_target_V;
  tfm.aas{1}.unplot_safe_V;
end

%% Just follow the path if vehicle is leader
if strcmp(veh.q, 'Leader')
  u = veh.followPath(tfm.hws{1});
  return
end

%% If there is a platoon, join the platoon if not already in the platoon
% Plot reachable set for second vehicle joining platoon
if strcmp(tfm.aas{2}.q, 'Free')
  tfm.aas{2}.plot_rel_target_V(tfm.qr_rel_target_V, ...
    tfm.hws{1}.ps{1}.vehicles{1}, tfm.rtt);
  tfm.aas{2}.plot_safe_V(tfm.aas{1}, tfm.qr_qr_safe_V, tfm.safetyTime)
else
  tfm.aas{2}.unplot_rel_target_V
  tfm.aas{2}.unplot_safe_V
end

if strcmp(veh.q, 'Free')
  u = veh.joinPlatoon(tfm.hws{1}.ps{1}, tfm);
  return
end



%% If already in the platoon, simply follow it
u = veh.followPlatoon(tfm);
end