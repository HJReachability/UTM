function checkPWSafety_pl_pl_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('pl_pl_safe_V');

% Create planes and add them to TFM
pl1 = Plane([0 0 0]);
pl1.speed = 10;
pl2 = Plane([100 0 pi]);
pl2.speed = 10;
tfm.regVehicle(pl1);
tfm.regVehicle(pl2);

% Plot initial setup
figure;
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
  hold on
end
xlim([-10 110])
ylim([-60 60])
axis square

% Integration parameters
tMax = 10;
t = 0:tfm.dt:tMax;

% Integrate
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;

  for j = 1:length(tfm.aas)
    if safe(j)
      u = 0;
    else
      u = uSafe{j};
    end
    
    tfm.aas{j}.updateState(u, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  drawnow;

end
end