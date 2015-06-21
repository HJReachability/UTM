function plotMergeHighwayV(obj, target)
% function plotMergeHighwayV(obj, target)
%
% Plots the zero sublevel set of value function for merging onto the
% highway
%
% Inputs:  obj    - quadrotor object
%          target - target state that quadrotor is aiming to reach
%
% Mo Chen, 2015-06-21

% State dimensions for position and velocity
pdim = obj.pdim;
vdim = obj.vdim;

% Unpack constants
if ~isempty(obj.mergeHighwayV)
    mergeV = obj.mergeHighwayV;
else
    delete(obj.hmergeHighwayV);
    obj.hmergeHighwayV = [];    
    return
end

tau = mergeV.tau;
g = mergeV.g;
datax = mergeV.datax;
datay = mergeV.datay;

% ----- Construct grid -----
% Position domain should cover all grid positions around the OTHER vehicle
% since p = [px py] indicates that this vehicle is at (px, py) where the
% origin is centered around the other vehicle
%
% Velocity domain should cover a thin layer around current relative
% velocity

% Slice to visualize

max_domain_size = 1;
domain_thickness = 3.1;
xmin = zeros(4,1);
xmin(1) = max_domain_size*g.min(1);
xmin(2) = obj.x(vdim(1)) - domain_thickness*g.dx(2);
xmin(3) = max_domain_size*g.min(1);
xmin(4) = obj.x(vdim(2)) - domain_thickness*g.dx(2);

xmax = zeros(4,1);
xmax(1) = max_domain_size*g.max(1);
xmax(2) = obj.x(vdim(1)) + domain_thickness*g.dx(2);
xmax(3) = max_domain_size*g.max(1);
xmax(4) = obj.x(vdim(2)) + domain_thickness*g.dx(2);

% Compute value for V(t,x) on the relative velocity slice and project down
% to 2D
[~, ~, g, value, ~] = recon2x2D(tau, g, datax, g, datay, [xmin xmax], inf);
xs = obj.x(vdim);

% Shift the grid!!!
[g2D, value2D] = proj2D(g, [0 1 0 1], g.N([1 4]), value, xs);
g2Dt.dim = g2D.dim;
g2Dt.min = g2D.min + target';
g2Dt.max = g2D.max + target';
g2Dt.N = g2D.N;
g2Dt.bdry = g2D.bdry;
g2Dt = processGrid(g2Dt);

% Plot result
if isempty(obj.hmergeHighwayV) || ~isvalid(obj.hmergeHighwayV)
    [~, obj.hmergeHighwayV] = contour(g2Dt.xs{1}, g2Dt.xs{2}, value2D, [0 0], ...
        'lineStyle', ':', 'linewidth', 2);
else
    obj.hmergeHighwayV.XData = g2Dt.xs{1};
    obj.hmergeHighwayV.YData = g2Dt.xs{2};
    obj.hmergeHighwayV.ZData = value2D;
end

% Color
if isempty(obj.hpxpyhist.Color)
    obj.hmergeHighwayV.LineColor = 'r';
else
    obj.hmergeHighwayV.LineColor = obj.hpxpyhist.Color;
end

end