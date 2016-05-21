function first_endpt_test()
%% Load path
load('bay_area_raw_paths', 'spath');
i = randi(length(spath));
path = spath{i};

%% Compute first endpoint
x0 = path(2,1);
y0 = path(1,1);

thetas = atan2(y0-path(2,2:end), x0-path(1,2:end));
ind = first_endpt(path);

disp(['Index of first endpoint: ' num2str(ind)])

%% Load images
% Image indices
imInd1 = 150:800;
imInd2 = 850:1500;

% Picture of Bay Area with city labels
dots = imread('bayAreaLabels.png');
dots = dots(imInd1, imInd2, :);

%% Grid
g.dim = 2;
g.min = -1;
g.max = 1;
g.N = size(dots,1);
g.bdry = @addGhostExtrapolate;
g = processGrid(g);

%% Plot results
f=figure;
subplot(1,2,1)
plot(thetas); hold on;
plot(ind, thetas(ind), 'ro')

subplot(1,2,2)
% Bay area map with city names
imagesc(g.vs{1}, g.vs{2}, dots); hold on
plot(path(2, :), path(1, :), 'r.')
plot(path(2, ind), path(1, ind), 'ro')

% Figure window size
f.Position(3:4) = [1043 420];

end