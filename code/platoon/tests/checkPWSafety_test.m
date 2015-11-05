function checkPWSafety_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('qr_qr_safeV');

% Add quadrotors to TFM
tfm.addActiveAgents(Quadrotor([0 3 0 0]));
tfm.addActiveAgents(Quadrotor([20 -3 0 0]));

% Plot initial setup
figure;
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
  hold on
end
xlim([-1 21])
ylim([-11 11])
axis square

% Integration parameters
tMax = 15;
t = 0:tfm.dt:tMax;

% Integrate
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;

  
  for j = 1:length(tfm.aas)
    if safe(j)
      u = [0; 0];
    else
      u = uSafe{j};
    end
    
    tfm.aas{j}.updateState(u, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  drawnow;
end


end