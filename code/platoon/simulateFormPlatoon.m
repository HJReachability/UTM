function simulateFormPlatoon()
%% TFM
tfm = TFM;
tfm.computeRS('qr_rel_target_V');
tfm.computeRS('qr_abs_target_V');
tfm.computeRS('qr_qr_safe_V');

figure;

%% Highway
hw_length = 1000;
z0 = [0 0];
z1 = [hw_length 0];
hw = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw);
tep = [300 0]; % Target entry  point

% plot
hw.lpPlot;
hold on

%% Quadrotors
xs = 0:10:40;
ys = -100 + 200*rand(size(xs));
for j = 1:length(xs)
  tfm.addActiveAgents(Quadrotor([xs(j) 0 ys(j) 0]));
  tfm.aas{j}.plotPosition;
end
drawnow

%% Integration
tMax = 30;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;
  
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = tfm.aas{j}.getToPose(tfm, tep, 0);
    else
      u{j} = uSafe{j};
    end
    
    tfm.aas{j}.updateState(u{j}, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  
  drawnow
end
end