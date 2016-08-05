function Quad4DRelativeLiveness()
N = 75;
N = N * ones(4,1);
gMin = [-5; -5; -5; -5];
gMax = [5; 5; 5; 5];

% Acceleration bounds
aMax = [3; 3];
bMax = [3; 3];

% Time vector
dt = 0.01;
tMax = 2;
tau = 0:dt:tMax;

% Target set
tarLower = [-1; -inf; -1; -inf];
tarUpper = [1; inf; 1; inf];

% visualization
vslice = [2 3];

schemeData.grid = createGrid(gMin, gMax, N);
schemeData.dynSys = Quad4DCAvoid(zeros(4,1), aMax, bMax);
schemeData.uMode = 'max';
schemeData.dMode = 'min';

data0 = shapeRectangleByCorners(schemeData.grid, tarLower, tarUpper);

extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1 0 1 0];
extraArgs.plotData.projpt = vslice;

data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
TTR = TD2TTR(data, tau);
P = computeGradients(schemeData.g, TTR);

save('Quad4DRelativeLiveness.mat', 'schemeData', 'data', '-v7.3')
save('Quad4DRelativeLiveness_smaller.mat', 'schemeData', 'TTR', 'P', '-v7.3')
end