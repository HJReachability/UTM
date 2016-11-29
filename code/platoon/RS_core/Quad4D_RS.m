function Quad4D_RS(gN)
% Quad4DCAvoid_RS()

if nargin < 1
  gN = 51;
end

%% Common parameters
small = 0.01;
targetLower = [-0.2; -inf; -0.2; -inf];
targetUpper = [0.2; inf; 0.2; inf];

% Grid
gMin = [-2; -2; -2; -2];
gMax = [2; 2; 2; 2];
uMode = 'max';
gN = gN*ones(4,1);

% Time
tMax = 0.5;
dt = 0.01;
tau = 0:dt:tMax;

% Vehicle
uMin = -1;
uMax = 1;

%% Dynamical systems and subsystems
Xdims = [1 2];
Ydims = [3 4];

%% Grids and initial conditions
gX = createGrid(gMin(Xdims), gMax(Xdims), gN(Xdims));
gY = createGrid(gMin(Ydims), gMax(Ydims), gN(Ydims));

dataX0 = shapeRectangleByCorners(gX, targetLower(Xdims), ...
  targetUpper(Xdims));
dataY0 = shapeRectangleByCorners(gY, targetLower(Ydims), ...
  targetUpper(Ydims));

%% Additional solver parameters
sD_X.grid = gX;
sD_Y.grid = gY;

sD_X.dynSys = Quad4D([0;0], uMin, uMax, Xdims);
sD_Y.dynSys = Quad4D([0;0], uMin, uMax, Ydims);

sD_X.uMode = uMode;
sD_Y.uMode = uMode;

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

dataX = HJIPDE_solve(dataX0, tau, sD_X, 'none', extraArgs);
dataY = HJIPDE_solve(dataY0, tau, sD_Y, 'none', extraArgs);

%% Reconstruct lower-dimensional solutions
vfs.gs = {sD_X.grid; sD_Y.grid};
vfs.datas = {dataX; dataY};
vfs.tau = tau;
vfs.dims = {Xdims; Ydims};
vf = reconSC(vfs, gMin-small, gMax+small, 'end', 'max');

%% Visualize
vslice = [1 1.5];
[g2D, data2D] = proj(vf.g, vf.dataMin, [0 1 0 1], vslice);

figure
visSetIm(g2D, data2D)
end