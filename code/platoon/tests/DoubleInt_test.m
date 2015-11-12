function DoubleInt_test()
% DoubleInt_test()
%
% Tests the double integrator model in getting to a target state
addpath('..')

%% Target set and reachable set
target = [0 10];
[~, TTR_out] = di_abs_target_2D(target, 0);
figure;
contour(TTR_out.g.xs{1}, TTR_out.g.xs{2}, TTR_out.value, 0:0.2:15)
hold on

%% Plot initial coniditions and setup figure
plot(target(1), target(2), 'ro')
xlim([-60 60])
ylim([-15 15])

%% Plot right switching curve
tt = linspace(-5, 0, 50);
x0right = 0;
y0right = 10 - 1.5*TTR_out.g.dx(2);
x = 0.5 * 3 * tt.^2 + y0right*tt + x0right;
y = 3*tt + y0right;
plot(x, y, 'k-.')

%% Plot left switching curve
x0left = -1.5*TTR_out.g.dx(1);
y0left = 10;
x = 0.5 * 3 * tt.^2 + y0left*tt + x0left;
y = 3*tt + y0left;
plot(x, y, 'k-.')

%% Simulation parameters
tMax = 50;
dt = 0.1;
t = 0:dt:tMax;
N = 20; % Repeat N times
for n = 1:N
  %% Random initial state
  init_x = [-20+40*rand -15+30*rand]; 
  d = DoubleInt(init_x);
  d.plotPosVel;
  
  title('t=0')
  drawnow;
  
  % Integrate
  for i = 1:length(t)
    %% Check if already at target
    valuex = eval_u(TTR_out.g, TTR_out.value, d.x);
    if valuex < 0.25
      break;
    end
    
    %% Check if in band and synthesize controller
    if in_band([x0left y0left], [x0right y0right], d.uMax, d.x)
      u = d.uMax;
    else
      p = calculateCostate(TTR_out.g, TTR_out.grad, d.x);
      u = (p(2) >= 0) * d.uMin + (p(2) < 0) * d.uMax;
    end
    
    %% Update state
    d.updateState(u, dt);
    d.plotPosVel;
    
    title(['t=' num2str(t)])
    drawnow;
  end
end
end

function in = in_band(left, right, u, state)
% Given y, evaluate x to see if it's between the band defined by the left
% and right optimal curves
%
% Inputs: left, right - the left and right points (x(0), y(0)), i.e. the
%                       final points for the left and right switching
%                       curves
%         u           - maximum acceleration
%         state       - the point (x,y) to be checked
% Output: in          - boolean indicating whether state is between left
%                       and right

in = false;

if state(1) > opt_curve_x(left(1), left(2), u, state(2)) && ...
    state(1) < opt_curve_x(right(1), right(2), u, state(2))
  in = true;
end

end

function x = opt_curve_x(x0, y0, u, y)
% Takes the final point (x0, y0), acceleration u, and a query value y as
% input, and outputs the corresponding value x that lies on the optimal
% curve determined by (x0, y0)

% Parametric form of the curve:
% x = 0.5 * u * t.^2 + y0*t + x0;
% y = u*t + y0;
% Take y as input ans solve for x

t = (y-y0)/u;
x = 0.5 * u * t.^2 + y0*t + x0;
end