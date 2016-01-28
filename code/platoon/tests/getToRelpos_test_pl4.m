function getToRelpos_test_pl4()
% getToRelpos_test()
%
% Tests a Plane4 heading to a position relative to a Plane4 going at
% constant velocity
% using the getToRelpos function
%
% Mahesh Vashishtha, 2016-01-25
addpath('..')

tfm = TFM;
tfm.computeRS('pl4_rel_target_V');

% Test N times...
N = 10;
for i = 1:N
  getToRelposSingle(tfm);
end

end

function getToRelposSingle(tfm)
% getToRelposSingle(tfm)
%
% A single random trial. The following a randomized:
%   initial position of the leader (50 distance away at an angle of
%                                   pos_theta)
%   initial velocity of the leader (10 at an angle of vel_theta)
%   target relative position relpos
%   initial follower state rand(4,1) (this is very small)
%
% Mo Chen, 2015-11-14

%% Preliminaries
debug = 1;
init_dist = 50; % Initial distance between the two vehicles


% Random relative position
relpos = -10 + 20*rand(2,1);

% Random leader and follower initial states
pos_theta = 2*pi*rand;
leader_init_pos = rotate2D([init_dist 0], pos_theta);

vel_theta = 2*pi*rand;
leader_init_vel = rotate2D([tfm.hw_speed 0], vel_theta);


leader = Plane4([leader_init_pos(1); leader_init_pos(2); ...
                 atan2(leader_init_vel(2),leader_init_vel(1));
                 norm([leader_init_vel(1),leader_init_vel(2)])]);
follower = Plane4([rand(3,1); rand() * 5 + 5]);

% Add vehicles to tfm
tfm.aas = {};
tfm.regVehicle(leader);
tfm.regVehicle(follower);

%% Initialize figures
figure;
% Plot initial positions
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
  hold on
end

% Plot path of leader
tMax = 30;
x0 = leader_init_pos(1);
x1 = leader_init_pos(1) + tMax*leader_init_vel(1);
y0 = leader_init_pos(2);
y1 = leader_init_pos(2) + tMax*leader_init_vel(2);
plot([x0 x1], [y0 y1], 'k--')

% plot (absolute) position of target
abs_pos = leader.getPosition + relpos;
ha = plot(abs_pos(1), abs_pos(2), 'ro');

grid on
xlim([min([x0 x1]) max([x0 x1])])
ylim([min([y0 y1]) max([y0 y1])])
axis equal
drawnow;

%% Integration
t = 0:tfm.dt:tMax;
u = zeros(2,2);
for i = 1:length(t)
  % Leader goes straight
  % u(:,1) is already [0; 0]
  
  % Follower joins leader
  u2 = follower.getToRelpos(leader, tfm, relpos, debug);
  
  if isempty(u2)
    u2 = [0; 0];
  end
  
  u(:,2) = u2;
  
  % Update position and plots
  for j = 1:length(tfm.aas)
    tfm.aas{j}.updateState(u(:,j), tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  
  abs_pos = leader.getPosition + relpos;
  ha.XData = abs_pos(1);
  ha.YData = abs_pos(2);  
  drawnow;
end
end