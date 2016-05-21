clear all;
close all;

% ----- Grid -----
g.dim = 2;
g.min = -1;
g.max = 1;
g.N = 201;
g.bdry = @addGhostExtrapolate;
g2 = g;
g = processGrid(g);

% ----- Load images -----
% Image indices
imInd1 = 150:800;
imInd2 = 850:1500;

% Picture of Bay Area with city labels
dots = imread('bayAreaLabels.png');
dots = dots(imInd1, imInd2, :);

% Bay Area converted to different region types
colored = imread('bayAreaRegions.png');
colored = colored(imInd1, imInd2, :);

g2.N = size(dots,1);
g2 = processGrid(g2);

% ----- Plot raw data -----
f1 = figure;

% Bay area map with city names
subplot(1,2,1), imagesc(g2.vs{1}, g2.vs{2}, dots); hold on

% Compute speed profile based on region types
speed = zeros(size(colored));

b = 4;  % Region speed factor (tuning parameter)
speed(colored==1) = b^2;   % Water
speed(colored==0) = b^0;   % City
speed(colored==2) = b^1;   % Other (trees, rocks, mountains)

% Plot speed profile
subplot(1,2,2), imagesc(g2.vs{1}, g2.vs{2}, speed); hold on

% Figure window size
f1.Position(3:4) = [1043 420];

% ----- Airports -----
SFO = [0.5785 -0.3354];
rSFO = 0.3;
obsSFO = sqrt((g2.xs{1}-SFO(1)).^2 + (g2.xs{2}-SFO(2)).^2) - rSFO;

OAK = [0.3231 0.0006];
rOAK = 0.2;
obsOAK = sqrt((g2.xs{1}-OAK(1)).^2 + (g2.xs{2}-OAK(2)).^2) - rOAK;

obs = shapeUnion(obsSFO, obsOAK);

speed(obs<=0) = b^-1;

% Plot obstacles
subplot(1,2,1)
contour(g2.xs{2}, g2.xs{1}, obs, [0 0], 'k-', 'linewidth', 2); 
plot(SFO(2), SFO(1), 'k+', 'linewidth', 2)
plot(OAK(2), OAK(1), 'k+', 'linewidth', 2)
axis(g.axis)
axis square

subplot(1,2,2)
contour(g2.xs{2}, g2.xs{1}, obs, [0 0], 'k-' , 'linewidth', 2); 
plot(SFO(2), SFO(1), 'k+', 'linewidth', 2)
plot(OAK(2), OAK(1), 'k+', 'linewidth', 2)
axis(g.axis)
axis square

% ----- Compute time to reach from "warehouse" in Concord -----
target = [-0.4338 0.4185];  % Concord
u = compute_value(g, target, speed); 

% ----- Paths to warehouse from different cities
IC = [0.1354 -0.4431; ...   % San Francisco
    -0.7877 -0.08;          % Vallejo
    -0.4215 -0.6862;        % San Rafael
    -0.1169 -0.1354;        % Berkeley
    -0.5108 0.9169;         % Antioch
    0.4369 0.3108;          % Hayward
    0.5908 -0.5938;         % Pacifica
    0.7323 -0.2338;         % San Mateo
    0.7723 0.5169];         % Fremont

P = extractCostates(g,u, true);   % Gradient of value function

% Compute shortest paths to target
spath = cell(size(IC,1),1);
for s = 1:length(spath)
    spath{s} = shortestPathP(g, P, u, IC(s,:), speed);
end

% ----- Plot results -----
f2 = figure;

% Shortest paths on top of map
subplot(1,2,1)
image(g2.vs{1}, g2.vs{2},dots); hold on
for s = 1:length(spath)
    plot(spath{s}(2,:), spath{s}(1,:),'r-','linewidth', 1.5)
end

% Airports
plot(SFO(2), SFO(1), 'k+', 'linewidth', 2)
plot(OAK(2), OAK(1), 'k+', 'linewidth', 2)
contour(g2.xs{2}, g2.xs{1}, obs, [0 0], 'k-', 'linewidth', 2)

axis(g.axis)
axis square
title('Bay Area Map, Shortest Paths')

% Shortest paths on top of speed profile and value function
subplot(1,2,2)
imagesc(g2.vs{1}, g2.vs{2}, speed); hold on
contour(g.xs{2},g.xs{1},u,0:0.04:1.5,'color','g')
for s = 1:length(spath)
    plot(spath{s}(2,:), spath{s}(1,:),'r-','linewidth', 1.5)
end

% Airports
plot(SFO(2), SFO(1), 'k+', 'linewidth', 2)
plot(OAK(2), OAK(1), 'k+', 'linewidth', 2)
contour(g2.xs{2}, g2.xs{1}, obs, [0 0], 'k-', 'linewidth', 2)


axis(g.axis)
axis square
title('Speed Profile, Shortest Paths, Value Function')

% Figure window size
f2.Position(3:4) = [1043 420];