function Quad4DSafety()
N = 71;
N = N * ones(4,1);
gMin = [-1; -1; -1; -1];
gMax = [1; 1; 1; 1];

% Acceleration bounds
aMax = [3; 3];
bMax = [3; 3];

% Time vector
dt = 0.01;
tMax = 2;
tau = 0:dt:tMax;

% Target set
tarLower = [-0.1; -0.1; -0.1; -0.1];
tarUpper = [0.1; 0.1; 0.1; 0.1];

% visualization
vslice = [0 0];

schemeData.grid = createGrid(gMin, gMax, N);
schemeData.dynSys = Quad4DCAvoid(zeros(4,1), aMax, bMax);
schemeData.uMode = 'min';
schemeData.dIn = {0; 0}; % Either use this or set bMax to [0; 0];

% Compute!
data0 = shapeRectangleByCorners(schemeData.grid, tarLower, tarUpper);

extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1 0 1 0];
extraArgs.plotData.projpt = vslice;

data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
TTR = TD2TTR(data, tau);
P = computeGradients(schemeData.g, TTR);

save('Quad4DSafety.mat', 'schemeData', 'data', '-v7.3')
save('Quad4DSafety_smaller.mat', 'schemeData', 'TTR', 'P', '-v7.3')
end