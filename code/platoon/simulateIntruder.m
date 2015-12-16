function simulateIntruder()
% simulateFormPlatoon()
%
% Simulates 5 quadrotors forming a single platoon on a highway

%% TFM
tfm = TFM;
tfm.computeRS('qr_qr_safe_V');

%% Highways
theta = 2*pi*rand;
hw_length = 300;
z0b = hw_length*[-0.25 0];
z1b = hw_length*[1 0];

% Highway 1
z0 = rotate2D(z0b, theta);
z1 = rotate2D(z1b, theta);
hw = Highway(z0, z1, tfm.hw_speed);
tfm.addHighway(hw);

% plot
figure;
hw.lpPlot;
hold on

%% Quadrotors
% Platoon 1 (4 vehicles)
x0 = 0;
xs = x0 + 3*tfm.ipsd: -tfm.ipsd: x0;
ys = zeros(size(xs));
xs_ys = rotate2D([xs; ys], theta);
xs = xs_ys(1,:);
ys = xs_ys(2,:);

v = rotate2D([10 0], theta);
for j = 1:length(xs)
  q = Quadrotor([xs(j) v(1) ys(j) v(2)]);
  tfm.regVehicle(q);
  if j == 1
    p1 = Platoon(q, hw, tfm);
  else
    p1.insertVehicle(q, j);
  end
end

% Intruder
ih = pi/4 + 6*pi/4*rand; % initial heading of intruder
mp = [50 0]; % meeting point between intruder and platoon
ip = mp - mp(1)*[cos(ih) sin(ih)]; % initial position
iv = 10*[cos(ih) sin(ih)]; % initial velocity
ip = rotate2D(ip, theta);
iv = rotate2D(iv, theta);
qi = Quadrotor([ip(1) iv(1) ip(2) iv(2)]);
tfm.regVehicle(qi);

for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
end
keyboard
title('t=0')
axis square
drawnow

%% Integration
tMax = 23;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;
  
  % TFM vehicles
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = controlLogic(tfm, tfm.aas{j});
    else
      u{j} = uSafe{j};
    end

    % Override control for intruder
    if tfm.aas{j} == qi
      u{j} = [0 0];
    end
    
    tfm.aas{j}.updateState(u{j}, tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  
  title(['t=' num2str(t(i))])
  drawnow

end

tfm.printBreadthFirst;
end

function u = controlLogic(tfm, veh)

if strcmp(veh.q, 'Free')
  u = [];
  return
end

if strcmp(veh.q, 'Leader')
  u = veh.followPath(veh.p.hw);
  return
end

% If already in the platoon, simply follow it
u = veh.followPlatoon(tfm);
end