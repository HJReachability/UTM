function [grids, datas, tau] = quad_rel_target_2D(x, visualize)
% function [grids, datas, tau] = quad_rel_target_2D(x, visualize)
%
% The relative coordinate dynamics are
%
% \dot x      = v_r = v1 - v2
% \dot v_r    = u1 - u2
%
% where input u2 is approximately 0 (staying on the highway)
%	      input u1 is trying to hit the target.
%
% Inputs: x         - target relative state
%         visualize - whether to plot the computation



if nargin<2
  visualize = 1;
end

if nargin<1
  x = [0 0 0 0];
end

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 15;                    % End time.
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
% Problem Parameters.
u1Max = 3;
u1Min = -3;
u2Max = 0;
u2Min = 0;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 51;  %changed by AKA from 81 to 41

% Create the grid.
g1.dim = 2;                              % Number of dimensions
g1.min = [ x(1)-45 ; -20 ];     % Bounds on computational domain
g1.max = [ x(1)+45 ; 20 ];
g1.bdry = @addGhostExtrapolate;
g1.N = [ Nx; ceil(Nx/(g1.max(1)-g1.min(1))*(g1.max(2)-g1.min(2)))];
g1 = processGrid(g1);

% Create the grid.
g2.dim = 2;                             % Number of dimensions
g2.min = [ x(3)-45 ; -20 ];     % Bounds on computational domain
g2.max = [ x(3)+45 ; 20 ];
g2.bdry = @addGhostExtrapolate;
g2.N = [ Nx; ceil(Nx/(g2.max(1)-g2.min(1))*(g2.max(2)-g2.min(2)))];
g2 = processGrid(g2);

% ----------------- Target -----------------
% Below minimum relative distance, small relative velocity, any velocity in x
datax = shapeRectangleByCorners(g1, [x(1); x(2)]-1.1*g1.dx, [x(1); x(2)]+1.1*g1.dx);

% Below minimum relative distance, small relative velocity, any velocity in x
datay = shapeRectangleByCorners(g2, [x(3); x(4)]-1.1*g2.dx, [x(3); x(4)]+1.1*g2.dx);

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;
% schemeData.grid1 = g1;
% schemeData.grid2 = g2;

% The Hamiltonian and partial functions need problem parameters.
schemeData.u1Max = u1Max;
schemeData.u1Min = u1Min;
schemeData.u2Max = u2Max;
schemeData.u2Min = u2Min;

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
accuracy = 'veryHigh';

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.9, 'stats', 'off');

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
% Initialize Display
if visualize
  f = figure;
  
  [~, h1] = contour(g1.xs{1}, g1.xs{2}, datax, [0 0],'r'); hold on
  contour(g1.xs{1}, g1.xs{2}, datax, [0 0],'r--');
  
  [~, h2] = contour(g2.xs{1}, g2.xs{2}, datay, [0 0],'b');
  contour(g2.xs{1}, g2.xs{2}, datay, [0 0],'b--');
  
  xlabel('x')
  ylabel('v')
  
  drawnow;
end
% return
%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
tau = tNow;
while(tMax - tNow > small * tMax)
  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tPlot) ];
  
  % Reshape data array into column vector for ode solver call.
  y0 = datax(:,:,end);
  y0 = y0(:);
  schemeData.grid = g1;
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  datax = cat(3, datax, reshape(y, g1.shape));
  
  % Reshape data array into column vector for ode solver call.
  y0 = datay(:,:,end);
  y0 = y0(:);
  schemeData.grid = g2;
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  datay = cat(3, datay, reshape(y, g2.shape));
  
  tNow = t(end);
  tau = cat(1, tau, tNow);
  
  % Create new visualization.
  if visualize
    h1.ZData = datax(:,:,end);
    h2.ZData = datay(:,:,end);
  end
  
  drawnow;
end

% Compact results into cell structures
grids = {g1; g2};
datas = {datax; datay};

end

%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = HamFunc(t, data, deriv, schemeData)
% HamFunc: analytic Hamiltonian for collision avoidance.
%
% hamValue = HamFunc(t, data, deriv, schemeData)
%
% This function implements the hamFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
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

checkStructureFields(schemeData, 'u1Max','u1Min', 'u2Max','u2Min', 'grid');

grid = schemeData.grid;

u1Max = schemeData.u1Max;
u1Min = schemeData.u1Min;
u2Max = schemeData.u2Max;
u2Min = schemeData.u2Min;


% Dynamics:
% \dot{x}_1 = v_1
% \dot{v}_1 = u_1

% \dot{x}_2 = v_2 (= constant)
% \dot{v}_2 = 0

% \dot{x}_r = v_r
% \dot{v}_r = u_1

% quadrotor 1 minimizes value, quadrotor 2 maximizes value
hamValue = deriv{1} .* grid.xs{2} + ...
  (deriv{2}>=0) .* (deriv{2}) * u1Min + ...
  (deriv{2}<0) .* (deriv{2}) * u1Max + ...
  (-deriv{2}>=0) .* (-deriv{2}) * u2Max + ...
  (-deriv{2}<0) .* (-deriv{2}) * u2Min;

% backwards reachable set
hamValue = -hamValue;
end

%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% PartialFunc: Hamiltonian partial fcn.
%
% It calculates the extrema of the absolute value of the partials of the
%   analytic Hamiltonian with respect to the costate (gradient).
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


checkStructureFields(schemeData, 'u1Max','u1Min', 'u2Max','u2Min', 'grid');

grid = schemeData.grid;


u1Max = schemeData.u1Max;
u1Min = schemeData.u1Min;
u2Max = schemeData.u2Max;
u2Min = schemeData.u2Min;

switch dim
  case 1
    alpha = abs(grid.xs{2});
    
  case 2
    alpha = max(abs([u1Min u1Max])) + max(abs([u2Min u2Max]));
    
  otherwise
    error([ 'Partials only exist in dimensions 1-2' ]);
end
end