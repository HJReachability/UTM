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

% ----- Load path -----
load('bay_area_raw_paths')

i = randi(length(spath));
raw_path = spath{i};

% Compute indices
pts = path2hws(raw_path);

% ----- Load images -----
% Image indices
imInd1 = 150:800;
imInd2 = 850:1500;

% Picture of Bay Area with city labels
dots = imread('bayAreaLabels.png');
dots = dots(imInd1, imInd2, :);

g2.N = size(dots,1);
g2 = processGrid(g2);

% ----- Plot -----
figure;
image(g2.vs{1}, g2.vs{2},dots); hold on
plot(raw_path(2,:), raw_path(1,:), 'b-')
plot(pts(2,:), pts(1,:), 'ro-')
axis square