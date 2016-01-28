function [data, g, tau] = pl4_pl4_collision_2D(d, speed, visualize)
% function [dataC, g2D, tau] = quad2Dcollision(d, visualize)
%
% Computes collision reachable set for 4D relative quadrotor dynamics by
% first computing 2D reachable sets in each axis and the reconstructing the
% 4D reachable set
%
% The relative coordinate dynamics in each axis is
%
% \dot x_r = v_r (= v1 - v2)
% \dot v_r = u1 - u2
%
% where input u2 trying to avoid the target and
%       input u1 trying to hit the target.
%
% Inputs:
%   d         - separation distance (default = 2)
%   visualize - whether to visualize the 2D reachable set (default = 1)
%
% Outputs:
%   dataC - 2D reachable sets in the x and y directions
%   g2D   - 2D grids in the x and y directions
%   tau   - time vector
%
% Mo Chen, 2015-07-14

%--------------------------------------------------------------------------
% You will see many executable lines that are commented out.
%   These are included to show some of the options available; modify
%   the commenting to modify the behavior.

if nargin < 1
  d = 5;
end

if nargin < 2
  speed = 10;
end

if nargin < 3
  visualize = 1;
end


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
% Problem Parameters.
u1Max = sqrt(3^2+(2*pi/10*12)^2);
u1Min = -u1Max;
u2Max = sqrt(3^2+(2*pi/10*12)^2);
u2Min = -u2Max;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 81;

% Create the grid.
g.dim = 2;                           % Number of dimensions
g.min = [-80; -2.1*speed ];     % Bounds on computational domain
g.max = [ 80;  2.1*speed ];
g.bdry = @addGhostExtrapolate;
g.N = [ Nx; ceil(1.5*Nx/(g.max(1)-g.min(1))*(g.max(2)-g.min(2)))];
g = processGrid(g);

% ----------------- Target -----------------
% Below separation distance for any relative velocity
data = shapeRectangleByCorners(g, [-d; -inf], [d; inf]);
%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;

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

  [~, h1] = contour(g.xs{1}, g.xs{2}, data, [0 0],'r'); hold on
  contour(g.xs{1}, g.xs{2}, data, [0 0],'r--');
  
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
  y0 = data(:,:,end);
  y0 = y0(:);
  schemeData.grid = g;
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  data = cat(3, data, reshape(y, g.shape));
  
  tNow = t(end);
  tau = cat(1, tau, tNow);
  
  % Create new visualization.
  if visualize
    delete(h1)
      [~, h1] = contour(g.xs{1}, g.xs{2}, data(:,:,end), [0 0],'r'); hold on

    title(['t=' num2str(tNow)]);
  end
  
  drawnow;
end

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
% \dot{x}_r = v_r
% \dot{v}_r = u_2 - u_1

% quadrotor 1 maximizes value, quadrotor 2 minimizes value
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