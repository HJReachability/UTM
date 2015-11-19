function followPlatoon_test()
addpath('..')

%% TFM
tfm = TFM;
tfm.computeRS('qr_rel_target_V');

%% Highway
hw = Highway([-50 0], [250 0]);
tfm.addHighway(hw);

%% Vehicles and platoon
leader = Quadrotor([0 10 0 0]);
follower2 = Quadrotor([-tfm.ipsd 10 0 0]);
follower2.x = follower2.x - 2 + 4*rand(4,1);
follower3 = Quadrotor([-2*tfm.ipsd 10 0 0]);
follower3.x = follower3.x - 2 + 4*rand(4,1);
follower4 = Quadrotor([-3*tfm.ipsd 10 0 0]);
follower4.x = follower4.x - 2 + 4*rand(4,1);
follower5 = Quadrotor([-4*tfm.ipsd 10 0 0]);
follower5.x = follower5.x - 2 + 4*rand(4,1);

platoon = Platoon(leader, hw, tfm);
platoon.insertVehicle(follower2, 2);
platoon.insertVehicle(follower3, 3);
platoon.insertVehicle(follower4, 4);
platoon.insertVehicle(follower5, 5);

%% Figure
figure;
leader.plotPosition;
hold on
follower2.plotPosition;
follower3.plotPosition;
follower4.plotPosition;
follower5.plotPosition;
hw.lpPlot;
ylim([-20 20])
title('t=0')
drawnow

%% Simulate
tMax = 20;
t = tfm.dt:tfm.dt:tMax;

uLeader = [0 0];
for i = 1:length(t)
  leader.updateState(uLeader, tfm.dt);
  
  [~, pPh_abs] = platoon.phantomPosition(tfm.ipsd, 2);
  uFol = follower2.simpleLinFB(uLeader, pPh_abs, leader.getVelocity);
  follower2.updateState(uFol, tfm.dt);

  [~, pPh_abs] = platoon.phantomPosition(tfm.ipsd, 3);
  uFol = follower3.simpleLinFB(uLeader, pPh_abs, leader.getVelocity);
  follower3.updateState(uFol, tfm.dt);
  
  [~, pPh_abs] = platoon.phantomPosition(tfm.ipsd, 4);
  uFol = follower4.simpleLinFB(uLeader, pPh_abs, leader.getVelocity);
  follower4.updateState(uFol, tfm.dt);
  
  [~, pPh_abs] = platoon.phantomPosition(tfm.ipsd, 5);
  uFol = follower5.simpleLinFB(uLeader, pPh_abs, leader.getVelocity);
  follower5.updateState(uFol, tfm.dt);
  
  leader.plotPosition;
  follower2.plotPosition;
  follower3.plotPosition;
  follower4.plotPosition;
  follower5.plotPosition;
  
  title(['t=' num2str(t(i))])
  drawnow
end
end