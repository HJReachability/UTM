function simulateFormPlatoon()
% simulateFormPlatoon()
%
% Simulates 5 quadrotors forming a single platoon on a highway

%% TFM
tfm = TFM;
tfm.computeRS('qr_rel_target_V');
tfm.computeRS('qr_abs_target_V');
tfm.computeRS('qr_qr_safe_V');

%% Highway
theta = 2*pi*rand;
hw_length = 500;
z0 = [0 0];
z1 = hw_length*[1 0];
z1 = rotate2D(z1, theta);
hw = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw);

% plot
figure;
hw.lpPlot;
hold on

%% Quadrotors
xs = 0:25:100;
ys = sign(rand(size(xs))-0.5).*(125 - xs);
xs_ys = rotate2D([xs; ys], theta);
xs = xs_ys(1,:);
ys = xs_ys(2,:);
for j = 1:length(xs)
  tfm.regVehicle(Quadrotor([xs(j) 0 ys(j) 0]));
  tfm.aas{j}.plotPosition;
end
title('t=0')
drawnow

%% Target entry points
tep = rotate2D([150 0], theta);

%% Integration
tMax = 30;
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

end


end

function u = controlLogic(tfm, veh, tep)
% If no platoon has been formed, then try to go on the highway to form one
if isempty(tfm.hws{1}.ps)
  u = veh.goOnHighway(tfm.hws{1}, tep, tfm);
  return
end

if strcmp(veh.q, 'Leader')
  u = veh.followPath(tfm.hws{1});
  return
end

% If there is a platoon, join the platoon if not already in the platoon
if strcmp(veh.q, 'Free')
  u = veh.joinPlatoon(tfm.hws{1}.ps{1}, tfm);
  return
end

% If already in the platoon, simply follow it
u = veh.followPlatoon(tfm);
end