function checkPWSafety_qr_qr_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('qr_qr_safe_V');

N = 10;
for i = 1:N
  checkPWSafety_qr_qr_single(tfm)
end

end

function checkPWSafety_qr_qr_single(tfm)

theta = 2*pi*rand;
dist2 = 50;
p1 = -rotate2D([dist2 0], theta) + rand(1,2);
p2 = rotate2D([dist2 0], theta);
v1 = rotate2D([10 0], theta);

% Add quadrotors to TFM
tfm.aas = {};
tfm.regVehicle(Quadrotor([p1(1) v1(1) p1(2) v1(2)]));
tfm.regVehicle(Quadrotor([p2(1) -v1(1) p2(2) -v1(2)]));

% Plot initial setup
figure;
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
  hold on
end
xlim([-dist2 dist2])
ylim([-dist2 dist2])
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
    
    % Plot safety reachable set for the other vehicle
    tfm.aas{j}.plot_safe_V(tfm.aas{~(j-1)+1}, tfm.qr_qr_safe_V, ...
      tfm.safetyTime);
  end
  drawnow;
end
end