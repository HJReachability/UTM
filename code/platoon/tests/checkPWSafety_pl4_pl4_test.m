function checkPWSafety_pl4_pl4_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('pl4_pl4_safe_V');


N = 10;
for i = 1:N
  checkPWSafety_pl4_pl4_single(tfm)
end

end

function checkPWSafety_pl4_pl4_single(tfm)

theta = 2*pi*rand;
dist2 = 50;
p1 = -rotate2D([dist2 0], theta) + rand(1,2);
p2 = [-p1(1) -p1(2)];
v1 = rotate2D([10 0], theta);


% Create Plane4 objects and add them to TFM
tfm.aas = {};
tfm.regVehicle(Plane4([p1(1) p1(2) atan2(v1(2),v1(1)) norm([v1(1) v1(2)])]));
tfm.regVehicle(Plane4([p2(1) p2(2) atan2(-v1(2),-v1(1)) norm([v1(1) v1(2)])]));

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
    
    % Plot safety reachable set for the other vehicle
    tfm.aas{j}.plot_safe_V(tfm.aas{~(j-1)+1}, tfm.pl4_pl4_safe_V, ...
      tfm.safetyTime);    
  end
  drawnow;

end
end