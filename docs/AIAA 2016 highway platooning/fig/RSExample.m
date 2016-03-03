function RSExample()

onlyTarget = 0;

g.dim = 2;
g.N = 101;
g.min = [-10; -10];
g.max = [10; 10];
g.bdry = @addGhostExtrapolate;
g = processGrid(g);

x = g.xs{1};
y = g.xs{2};
R = 4;

figure
%%
T = sqrt(x.^2 + y.^2) - R;
surf(x,y,T, 'facecolor', 'r', 'facealpha', 0.3, 'edgecolor', 'none')
hold on
t = linspace(0, 2*pi, 101);
plot(R*cos(t), R*sin(t), 'r-', 'linewidth', 4)

%%
surf(x, y, zeros(size(x)), 'facecolor', [0 0.25 0], ...
  'facealpha', 0.5, 'edgecolor', 'none')

if onlyTarget
  xlabel('x')
  ylabel('y')
  legend('l(x)','Target set', 'Zero plane')
  return
end

%%
s = 5; % shift
% Shifted circle
C = sqrt((x-s).^2 + y.^2) - R;

% Rectangle
ledge = -x;
redge = x-s;
tedge = y-R;
bedge = -y-R;
rect = max(ledge, redge);
rect = max(rect, tedge);
rect = max(rect, bedge);

% Union
U = min(C, rect);
U = min(U, T);
U = signedDistanceIterative(g, U)/1.1;
surf(x,y,U, 'facecolor', 'b', 'facealpha', 0.3, 'edgecolor', 'none')
hold on

HedgeX = linspace(0, s, 50);
TedgeY = R*ones(size(HedgeX));
plot(HedgeX, TedgeY, 'b-', 'linewidth', 4)
BedgeY = -R*ones(size(HedgeX));
plot(HedgeX, BedgeY, 'b-', 'linewidth', 4)

t = linspace(pi/2, 3*pi/2, 50);
plot(R*cos(t), R*sin(t),  'b-', 'linewidth', 4)
t = linspace(-pi/2, pi/2, 50);
plot(s+R*cos(t), R*sin(t),  'b-', 'linewidth', 4)


xlabel('x')
ylabel('y')
legend('l(x)','Target set', 'Zero plane', 'V(t,x)', 'Reachable set')

end