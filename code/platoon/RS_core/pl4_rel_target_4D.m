function [g, data] = pl4_rel_target_4D(accuracy)
% [ data, g, data0 ] = pl4_rel_target_4D(accuracy)
%
% Input Parameters:
%
%   accuracy: Controls the order of approximations.
%     'low': Use odeCFL1 and upwindFirstFirst.
%     'medium': Use odeCFL2 and upwindFirstENO2 (default).
%     'high': Use odeCFL3 and upwindFirstENO3.
%     'veryHigh': Use odeCFL3 and upwindFirstWENO5.
%
% Output Parameters:
%
%   data: Implicit surface function at t_max.
%
%   g: Grid structure on which data was computed.
%
%   data0: Implicit surface function at t_0.

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 3;                    % End time.
plotSteps = 1;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
singleStep = 1;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

%---------------------------------------------------------------------------
% What level set should we view?
level = 0;

% Visualize the 3D reachable set.
displayType = 'surface';

% Pause after each plot?
pauseAfterPlot = 0;

% Delete previous plot before showing next?
deleteLastPlot = 1;

% Visualize the angular dimension a little bigger.
aspectRatio = [ 1 1 0.4 ];

% Plot in separate subplots (set deleteLastPlot = 0 in this case)?
useSubplots = 0;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 31;

% Create the grid.
g.dim = 4;
g.min = [  -10; -10; -pi; -5];
g.max = [ 10; 10; pi; 5];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic; @addGhostExtrapolate};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; ceil(Nx * (g.max(2) - g.min(2)) / (g.max(1) - g.min(1))); ...
  Nx-1; Nx];
% Need to trim max bound in \psi (since the BC are periodic in this dimension).
g.max(3) = g.max(3) * (1 - 1 / g.N(3));
g = processGrid(g);

% ----------------- Target -----------------
data = shapeSphere(g, [ 0; 0; 0; 0], 1.5*max(g.dx));
data0 = data;


%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.

% max angular velocity
schemeData.max_u1 = 2*pi/10;
schemeData.min_u1 = -2*pi/10;
% max acceleration
schemeData.max_u2 = 3;
schemeData.min_u2 = -3;

%---------------------------------------------------------------------------
% Choose degree of dissipation.

switch(dissType)
  case 'global'
    schemeData.dissFunc = @artificialDissipationGLF;
  case 'local'
    schemeData.dissFunc = @artificialDissipationLLF;
  case 'locallocal'
    schemeData.dissFunc = @artificialDissipationLLLF;
  otherwise
    error('Unknown dissipation function %s', dissFunc);
end

%---------------------------------------------------------------------------
if(nargin < 1)
  accuracy = 'medium';
end

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.9, 'stats', 'on');

% Choose approximations at appropriate level of accuracy.
switch(accuracy)
  case 'low'
    schemeData.derivFunc = @upwindFirstFirst;
    integratorFunc = @odeCFL1;
  case 'medium'
    schemeData.derivFunc = @upwindFirstENO2;
    integratorFunc = @odeCFL2;
  case 'high'
    schemeData.derivFunc = @upwindFirstENO3;
    integratorFunc = @odeCFL3;
  case 'veryHigh'
    schemeData.derivFunc = @upwindFirstWENO5;
    integratorFunc = @odeCFL3;
  otherwise
    error('Unknown accuracy level %s', accuracy);
end

if(singleStep)
  integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');
end

%---------------------------------------------------------------------------
% Restrict the Hamiltonian so that reachable set only grows.
%   The Lax-Friedrichs approximation scheme MUST already be completely set up.
innerFunc = schemeFunc;
innerData = schemeData;
clear schemeFunc schemeData;

% Wrap the true Hamiltonian inside the term approximation restriction routine.
schemeFunc = @termRestrictUpdate;
schemeData.innerFunc = innerFunc;
schemeData.innerData = innerData;
schemeData.positive = 0;

%---------------------------------------------------------------------------
% Initialize Display
f = figure;


% [g2D, data2D] = proj2D(g,data,[0 0 1 1],[0 10]);
% [~, h] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0]);
[g3D, data3D] = proj3D(g, data, [0 0 0 1], 0);
h = visualizeLevelSet(g3D, data3D, 'surface', 0);
camlight right
camlight left
hold on;
axis(g3D.axis);
drawnow;

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
while(tMax - tNow > small * tMax)
  % Reshape data array into column vector for ode solver call.
  y0 = data(:);
  
  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tPlot) ];
  
  % Take a timestep.
  [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  tNow = t(end);
  
  % Get back the correctly shaped data array
  data = reshape(y, g.shape);
  
  if(pauseAfterPlot)
    % Wait for last plot to be digested.
    pause;
  end
  
  % Get correct figure, and remember its current view.
  figure(f);
  [ view_az, view_el ] = view;
  
  % Move to next subplot if necessary.
  if(useSubplots)
    plotNum = plotNum + 1;
    subplot(rows, cols, plotNum);
  end
  
  % Create new visualization.
%   [~, data2D] = proj2D(g,data,[0 0 1 1],[0 10]);
%   h.ZData = data2D;
  [g3D, data3D] = proj3D(g, data, [0 0 0 1], 0);
  delete(h);
  h = visualizeLevelSet(g3D, data3D, 'surface', 0);

  % Restore view.
  view(view_az, view_el);
  
end

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = HamFunc(t, data, deriv, schemeData)
% HamFunc: analytic Hamiltonian
%
% hamValue = HamFunc(t, data, deriv, schemeData)
%
% It calculates the analytic Hamiltonian for such a flow field.
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   deriv	 Cell vector of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%
%   hamValue	 The analytic hamiltonian.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):
%
%   .grid	 Grid structure.

checkStructureFields(schemeData, 'grid',  'max_u1', 'max_u2', ...
  'min_u1', 'min_u2');

grid = schemeData.grid;
max_u1 = schemeData.max_u1;
min_u1 = schemeData.min_u1;
max_u2 = schemeData.max_u2;
min_u2 = schemeData.min_u2;


hamValue = deriv{1} .* grid.xs{4} .* cos(grid.xs{3}) +  ...
  deriv{2} .* grid.xs{4} .* sin(grid.xs{3}) + ...
  deriv{3} .* ((deriv{3}>=0) * min_u1 + ...
  (deriv{3}<0) * max_u1) + ...
  deriv{4} .* ((deriv{4}>=0) * min_u2 + ...
  (deriv{4}<0) * max_u2);
hamValue = -hamValue;


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% PartialFunc: Hamiltonian partial function
%
% alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   derivMin	 Cell vector of minimum values of the costate (\grad \phi).
%   derivMax	 Cell vector of maximum values of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%   dim          Dimension in which the partial derivatives is taken.
%
%   alpha	 Maximum absolute value of the partial of the Hamiltonian
%		   with respect to the costate in dimension dim for the
%                  specified range of costate values (O&F equation 5.12).
%		   Note that alpha can (and should) be evaluated separately
%		   at each node of the grid.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):
%
%   .grid	 Grid structure.

checkStructureFields(schemeData, 'grid','max_u1', 'max_u2', 'min_u1', 'min_u2');

grid = schemeData.grid;
max_u1 = schemeData.max_u1;
max_u2 = schemeData.max_u2;
min_u1 = schemeData.min_u1;
min_u2 = schemeData.min_u2;

switch dim
  case 1
    alpha = abs(grid.xs{4} .* cos(grid.xs{3}));
    
  case 2
    alpha = abs(grid.xs{4} .* sin(grid.xs{3}));
  case 3
    alpha = max(abs([max_u1 min_u1]));
  case 4
    alpha = max(abs([max_u2 min_u2]));
  otherwise
    error([ 'Partials only exist in dimensions 1-4' ]);
end
