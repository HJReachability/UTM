function Quad4DCAvoid_RS()
% Quad4DCAvoid_RS()

%% Common parameters
small = 0.01;
targetLower = [-0.2; -inf; -0.2; -inf];
targetUpper = [0.2; inf; 0.2; inf];

% Grid
gMin = [-2; -3; -2; -3];
gMax = [2; 3; 2; 3];
uMode = 'max';
dMode = 'min';
gN = [35; 55; 35; 55];

% Time
tMax = 1.5;
dt = 0.01;
tau = 0:dt:tMax;

% Vehicle
aMax = [1 1];
bMax = [1 1];

%% Dynamical systems and subsystems
Xdims = [1 2];
Ydims = [3 4];

%% Grids and initial conditions
g = createGrid(gMin, gMax, gN);
gX = createGrid(gMin(Xdims), gMax(Xdims), gN(Xdims));
gY = createGrid(gMin(Ydims), gMax(Ydims), gN(Ydims));

dataX0 = shapeRectangleByCorners(gX, targetLower(Xdims), ...
  targetUpper(Xdims));
dataY0 = shapeRectangleByCorners(gY, targetLower(Ydims), ...
  targetUpper(Ydims));

%% Additional solver parameters
sD_X.grid = gX;
sD_Y.grid = gY;

sD_X.dynSys = Quad4DCAvoid([0;0], aMax, bMax, 0, 0, Xdims);
sD_Y.dynSys = Quad4DCAvoid([0;0], aMax, bMax, 0, 0, Ydims);

sD_X.uMode = uMode;
sD_Y.uMode = uMode;

sD_X.dMode = dMode;
sD_Y.dMode = dMode;
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

dataX = HJIPDE_solve(dataX0, tau, sD_X, 'none', extraArgs);
dataY = HJIPDE_solve(dataY0, tau, sD_Y, 'none', extraArgs);

%% Reconstruct lower-dimensional solutions
vfs.gs = {sD_X.grid; sD_Y.grid};
vfs.datas = {dataX; dataY};
vfs.tau = tau;
vfs.dims = {Xdims; Ydims};
vf = reconSC(vfs, gMin-small, gMax+small, 'full', 'max');

%% Visualize
vslice = [3 2];
[g2D, data2D] = proj(vf.g, vf.dataMin, [0 1 0 1], vslice);

figure
visSetIm(g2D, data2D)

%% TTR
TTR = TD2TTR(vf.g, vf.data, vf.tau);

[g2D, data2D] = proj(vf.g, TTR, [0 1 0 1], vslice);

figure
visSetIm(g2D, data2D, 'k', 1)
end