function DoubleInt_test()
addpath('..')


% Target set and reachable set
target = [0 10];
[~, TTR_out] = di_abs_target_2D(target, 0);
figure;
contour(TTR_out.g.xs{1}, TTR_out.g.xs{2}, TTR_out.value, 0:0.2:15)
hold on

% Plot initial coniditions and setup figure
plot(target(1), target(2), 'ro')
xlim([-35 10])
ylim([-15 15])

% % Plot switching curve
% y = linspace(-15, 15, 100);
% tt = (10 - 1.5*TTR_out.g.dx(2) -y)/3;
% x = -0.5 * 3 * tt.^2;
% plot(x,y,'k--')

% % Plot upper left special trajectory
% tt = linspace(-5, 0, 50);
% y = 3*tt + 10 + 1.5*TTR_out.g.dx(2);
% x = 0.5 * 3 * tt.^2 + (10+1.5*TTR_out.g.dx(2))*tt - 1.5*TTR_out.g.dx(2);
% plot(x, y, 'k-.')

% Plot lower left special trajectory
tt = linspace(-5, 0, 50);
y = 3*tt + 10 - 1.5*TTR_out.g.dx(2);
x = 0.5 * 3 * tt.^2 + (10-1.5*TTR_out.g.dx(2))*tt - 1.5*TTR_out.g.dx(2);
plot(x, y, 'k-.')

% % Plot lower right special trajectory
% tt = linspace(-5, 0, 50);
% y = 3*tt + 10 - 1.5*TTR_out.g.dx(2);
% x = 0.5 * 3 * tt.^2 + (10-1.5*TTR_out.g.dx(2))*tt + 1.5*TTR_out.g.dx(2);
% plot(x, y, 'k-.')

% Simulation parameters
tMax = 50;
dt = 0.1;
t = 0:dt:tMax;
N = 10;
for n = 1:N
  init_x = [-30*rand -8 + 16*rand];
  d = DoubleInt(init_x);
  d.plotPosVelx;

  title('t=0')
  drawnow;
  
  for i = 1:length(t)
    valuex = eval_u(TTR_out.g, TTR_out.value, d.x);
    
    if valuex < 0.1
      break;
    end
    
    p = calculateCostate(TTR_out.g, TTR_out.grad, d.x);
    u = (p(2) >= -0.05) * d.uMin + (p(2) < -0.05) * d.uMax;
    d.updateState(u, dt);
    d.plotPosVelx;
    
    title(['t=' num2str(t)])
    drawnow;
  end
end