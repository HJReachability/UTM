clear all;
close all;

addpath('..')

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
g2.N = size(dots,1);
g2 = processGrid(g2);

f = figure;
subplot(1,2,2)
image(g2.vs{1}, g2.vs{2},dots); hold on
axis square

subplot(1,2,1)
image(g2.vs{1}, g2.vs{2},dots,'visible','off'); hold on
axis square


% ----- Load path -----
load('bay_area_raw_paths')

% Plot initial way points
pts = cell(length(spath),1);
colors = lines(length(spath));
for i = 1:length(spath)
  raw_path = spath{i};
  
  % Compute indices
  pts{i} = path2hws(raw_path);
  
  % ----- Plot -----
  subplot(1,2,1)
  plot(pts{i}(2,:), pts{i}(1,:), '.:', 'color', colors(i,:))
  axis square
end

% ----- Sparsify -----
threshold = 0.025;
sparse_pts = sparsify_paths(pts, threshold);
for i = 1:length(sparse_pts)
  subplot(1,2,2)
  plot(sparse_pts{i}(2,:), sparse_pts{i}(1,:), 'x-', ...
    'color', colors(i,:), 'linewidth', 2)
  
  subplot(1,2,1)
  plot(sparse_pts{i}(2,:), sparse_pts{i}(1,:), 'x-', ...
    'color', colors(i,:))
end

% Figure window size
f.Position(3:4) = [1043 420];