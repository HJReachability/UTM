function plotSafeV(obj, other, safeV, t)
% function plotSafeV(obj, other, t)
%
% Plots the safe region around other that obj must stay out of in order to
% be safe.
%
% Inputs: obj   - this quadrotor
%         other - other quadrotor
%         safeV - Reachable set 
%         t     - time horizon
%
% Mo Chen, 2015-06-21

% State dimensions for position and velocity
pdim = obj.pdim;
vdim = obj.vdim;

% Unpack constants
tau = safeV.tau;
g = safeV.g;
dataC = safeV.dataC;
dataS = safeV.dataS;

% ----- Construct grid -----
% Position domain should cover all grid positions around the OTHER vehicle
% since p = [px py] indicates that this vehicle is at (px, py) where the
% origin is centered around the other vehicle
%
% Velocity domain should cover a thin layer around current relative
% velocity


max_domain_size = 1;
domain_thickness = 1.6;
xmin = zeros(6,1);
% xmin(1) = max( max_domain_size*g.min(1), -apdiffx );
xmin(1) = max_domain_size*g.min(1);
xmin(2) = obj.x(vdim(1))-other.x(vdim(1)) - domain_thickness*g.dx(2);
xmin(3) = obj.x(vdim(1))                  - domain_thickness*g.dx(3);
% xmin(4) = max( max_domain_size*g.min(1), -apdiffy );
xmin(4) = max_domain_size*g.min(1);
xmin(5) = obj.x(vdim(2))-other.x(vdim(2)) - domain_thickness*g.dx(2);
xmin(6) = obj.x(vdim(2))                  - domain_thickness*g.dx(3);

xmax = zeros(6,1);
% xmax(1) = min( max_domain_size*g.max(1), apdiffx );
xmax(1) = max_domain_size*g.max(1);
xmax(2) = obj.x(vdim(1))-other.x(vdim(1)) + domain_thickness*g.dx(2);
xmax(3) = obj.x(vdim(1))                  + domain_thickness*g.dx(3);
% xmax(4) = min( max_domain_size*g.max(1), apdiffy );
xmax(4) = max_domain_size*g.max(1);
xmax(5) = obj.x(vdim(2))-other.x(vdim(2)) + domain_thickness*g.dx(2);
xmax(6) = obj.x(vdim(2))                  + domain_thickness*g.dx(3);
    
% Determine time horizon
if nargin<4
    % If the quadrotor is in a platoon AND the other quadrotor is in a
    % platoon AND (the platoons are the same OR one of the quadrotors is an
    % EmergLeader), then use internal separation time. 
    % Otherwise, use external separation time.
    if ~isempty(obj.p) && ~isempty(other.p) && ...
            (obj.p.ID == other.p.ID || ...
            strcmp(obj.q,'EmergLeader') || strcmp(other.q, 'EmergLeader'))
        t = obj.tauInt;
    else                     
        t = obj.tauExt;
    end
end

% Compute value for V(t,x) on the relative velocity slice and project down
% to 2D

% Collision reachable set: reconstruct the "max" problem
[~, ~, g6D, valueC, ~, ind] = recon2x3D(tau, g, dataC, g, dataC, [xmin xmax], t);

% Velocity reachable set: take union for the "min" problem
valueSx = eval_u(g, dataS(:,:,:,ind), [g6D.xs{1}(:) g6D.xs{2}(:) g6D.xs{3}(:)]);
valueSy = eval_u(g, dataS(:,:,:,ind), [g6D.xs{4}(:) g6D.xs{5}(:) g6D.xs{6}(:)]);
valueS = min(valueSx, valueSy);
valueS = reshape(valueS, g6D.shape);

% Overall safety set is the union of collision and velocity limit sets
value = min(valueC, valueS);

% Relative velocity slice
xs = zeros(4,1);
xs([1 3]) = obj.x(vdim)-other.x(vdim);
xs([2 4]) = obj.x(vdim);

% Take above relative velocity slice and shift the grid to be centered
% around "other"'s position
[g2D, value2D] = proj2D(g6D, [0 1 1 0 1 1], g6D.N([1 4]), value, xs);
g2Dt.dim = g2D.dim;
g2Dt.min = g2D.min + other.x(pdim);
g2Dt.max = g2D.max + other.x(pdim);
g2Dt.N = g2D.N;
g2Dt.bdry = g2D.bdry;
g2Dt = processGrid(g2Dt);

% Plot result
if isempty(obj.hsafeV{other.ID}) || ~isvalid(obj.hsafeV{other.ID})
    [~, obj.hsafeV{other.ID}] = contour(g2Dt.xs{1}, g2Dt.xs{2}, value2D, [0 0], ...
        'lineStyle', '--');
    if isempty(obj.hpxpyhist.Color)
        obj.hsafeV{other.ID}.Color = [0.5,0.5,0.5];
    else
        obj.hsafeV{other.ID}.Color = obj.hpxpyhist.Color;
    end
    
else
    % If plot already exists, simply update the plot
    obj.hsafeV{other.ID}.XData = g2Dt.xs{1};
    obj.hsafeV{other.ID}.YData = g2Dt.xs{2};
    obj.hsafeV{other.ID}.ZData = value2D;
end
end