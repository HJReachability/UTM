function test_pl4_target_RS()
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
  test_pl4_target_single(tfm);
end

end

function test_pl4_target_single(tfm)
% getToRelposSingle(tfm)
%
% A single random trial. The following a randomized:
%   initial position of the pursuer (50 distance away at an angle of
%                                   pos_theta)
%   initial velocity of the pursuer (10 at an angle of vel_theta)
%   target relative position relpos
%   initial follower state rand(4,1) (this is very small)
%
% Mo Chen, 2015-11-14

%% Preliminaries
debug = 1;
init_dist = 50; % Initial distance between the two vehicles


% Random relative position
target_state = [-10 + 20*rand(2,1); rotate2D([5; 5]+(rand(2,1) -[.5; .5]),2*pi*rand)];
target = Plane4(target_state);

% Random pursuer and follower initial states
pos_theta = 2*pi*rand;
pursuer_init_pos = rotate2D([init_dist 0], pos_theta);

vel_theta = 2*pi*rand;
pursuer_init_vel = rotate2D([tfm.hw_speed 0], vel_theta);


pursuer = Plane4([pursuer_init_pos(1); pursuer_init_pos(2); ...
                 atan2(pursuer_init_vel(2),pursuer_init_vel(1));
                 norm([pursuer_init_vel(1),pursuer_init_vel(2)])]);

% Add vehicles to tfm
tfm.aas = {};
tfm.regVehicle(target);
tfm.regVehicle(pursuer);


%% Initialize figures
figure;
% Plot initial positions
for j = 1:length(tfm.aas)
  tfm.aas{j}.plotPosition;
  hold on
end

% Plot path of pursuer
tMax = 30;
x0 = pursuer_init_pos(1);
x1 = pursuer_init_pos(1) + tMax*pursuer_init_vel(1);
y0 = pursuer_init_pos(2);
y1 = pursuer_init_pos(2) + tMax*pursuer_init_vel(2);
plot([x0 x1], [y0 y1], 'k--')

% plot (absolute) position of target


grid on
xlim([min([x0 x1]) max([x0 x1])])
ylim([min([y0 y1]) max([y0 y1])])
axis equal
drawnow;

%% Integration
t = 0:tfm.dt:tMax;

for i = 1:length(t)
   
  u2 = pursuer.getToRelpos(target, tfm, [0; 0], debug);
  
  if isempty(u2)
    u2 = [0; 0];
  end
  
   u(:,2) = u2;
   u(:,1) = [0; 0];
  
  % Update position and plots
  for j = 1:length(tfm.aas)
    tfm.aas{j}.updateState(u(:,j), tfm.dt);
    tfm.aas{j}.plotPosition;
  end
  
  abs_pos = pursuer.getPosition;
  ha.XData = abs_pos(1);
  ha.YData = abs_pos(2);  
  drawnow;
end
end