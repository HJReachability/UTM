function checkPWSafety_pl4_pl4_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('pl4_pl4_safe_V');

% Create Plane4 objects and add them to TFM
pl1 = Plane4([0 0 0 10]);
pl2 = Plane4([100 0 pi 10]);
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
      u = [0;0];
    else
      xi3 = tfm.aas{j}.x(3);
      xi4 = tfm.aas{j}.x(4);
      M = [cos(xi3) sin(xi3); -sin(xi3) / xi4 cos(xi3) / xi4];
      u = M  * uSafe{j};
    end
    
    tfm.aas{j}.updateState(u, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  drawnow;

end
end