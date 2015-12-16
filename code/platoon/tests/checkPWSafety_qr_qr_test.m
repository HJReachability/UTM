function checkPWSafety_qr_qr_test()
addpath('..')
addpath('../RS_core')

% Initialize TFM
tfm = TFM;
tfm.computeRS('qr_qr_safe_V');

N = 5;
for i = 1:N
  checkPWSafety_qr_qr_single(tfm, i, N)
end

end

function checkPWSafety_qr_qr_single(tfm, i, N)
disp(['Trial ' num2str(i)])
theta1 = 2*pi*rand;
theta2 = theta1 + 2*pi * i/(N+1);
dist2 = 50;
p1 = rotate2D([dist2 0], theta1) + rand(1,2);
p2 = rotate2D([dist2 0], theta2);
v1 = -rotate2D([10 0], theta1);
v2 = -rotate2D([10 0], theta2);

% Add quadrotors to TFM
tfm.aas = {};
tfm.regVehicle(Quadrotor([p1(1) v1(1) p1(2) v1(2)]));
tfm.regVehicle(Quadrotor([p2(1) v2(1) p2(2) v2(2)]));

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
  [safe, uSafe, safe_val] = tfm.checkAASafety;

  for j = 1:length(tfm.aas)
    if safe(j)
      u = [0; 0];
    else
      u = uSafe{j};
    end
    
    if j == 2
      u = [0; 0];
    end
    
    tfm.aas{j}.updateState(u, tfm.dt);
    tfm.aas{j}.plotPosition;
    
%     % Plot safety reachable set for the other vehicle
%     tfm.aas{j}.plot_safe_V(tfm.aas{~(j-1)+1}, tfm.qr_qr_safe_V, ...
%       tfm.safetyTime);
  end
  tfm.aas{1}.u
  
  tfm.aas{1}.plot_safe_V(tfm.aas{2}, tfm.qr_qr_safe_V, ...
      tfm.safetyTime)
%    disp(['Safety values: ' num2str(safe_val(1)), ', ' num2str(safe_val(2))])
%    if safe_val(1) < 8
%      keyboard
%    end
  drawnow;
end
end