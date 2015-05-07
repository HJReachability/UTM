function plotMergePlatoonV(obj, other, highway)
% function plotSafeV(obj, other, t)
% Plots the safe region around other that obj must stay out of in order to
% be safe.
%
% Inputs: obj   - this quadrotor
%         other - other quadrotor
%         safeV - Reachable set 
%         t     - time horizon
%

% State dimensions for position and velocity
pdim = obj.pdim;
vdim = obj.vdim;

% Unpack constants
if ~isempty(obj.mergePlatoonV)
    mergeV = obj.mergePlatoonV;
else
    delete(obj.hmergePlatoonV);
    obj.hmergePlatoonV = [];
    return
end

tau = mergeV.tau;
g1 = mergeV.g1;
g2 = mergeV.g2;
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
xmin(1) = max_domain_size*g1.min(1);
xmin(2) = obj.x(vdim(1))-other.x(vdim(1)) - domain_thickness*g1.dx(2);
xmin(3) = max_domain_size*g2.min(1);
xmin(4) = obj.x(vdim(2))-other.x(vdim(2)) - domain_thickness*g2.dx(2);

xmax = zeros(4,1);
xmax(1) = max_domain_size*g1.max(1);
xmax(2) = obj.x(vdim(1))-other.x(vdim(1)) + domain_thickness*g1.dx(2);
xmax(3) = max_domain_size*g2.max(1);
xmax(4) = obj.x(vdim(2))-other.x(vdim(2)) + domain_thickness*g2.dx(2);

% Compute value for V(t,x) on the relative velocity slice and project down
% to 2D
[~, ~, g4D, value, ~] = recon2x2D(tau, g1, datax, g2, datay, [xmin xmax], inf);
xs = obj.x(vdim)-other.x(vdim);

% figure;
% Shift the grid!!!
[g2D, value2D] = proj2D(g4D, [0 1 0 1], g4D.N([1 4]), value, xs);
g2Dt.dim = g2D.dim;
g2Dt.min = g2D.min + other.x(pdim);
g2Dt.max = g2D.max + other.x(pdim);
g2Dt.N = g2D.N;
g2Dt.bdry = g2D.bdry;
g2Dt = processGrid(g2Dt);

% Plot result
if isempty(obj.hmergePlatoonV) || ~isvalid(obj.hmergePlatoonV)
    [~, obj.hmergePlatoonV] = contour(g2Dt.xs{1}, g2Dt.xs{2}, value2D, [0 0], ...
        'lineStyle', ':', 'linewidth', 2);

else
    obj.hmergePlatoonV.XData = g2Dt.xs{1};
    obj.hmergePlatoonV.YData = g2Dt.xs{2};
    obj.hmergePlatoonV.ZData = value2D;
end

obj.hmergePlatoonV.LineColor = 'b';
end