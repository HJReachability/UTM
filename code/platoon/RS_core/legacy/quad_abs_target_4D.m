function [TD_out, TTR_out] = quad_abs_target_4D(x)
% [grids, datas, tau] = quad_abs_target_2D(x, visualize)
%
% Computes 2D liveness reachable set for merging onto the highway. These
% need to be reconstructed in 4D.
%
% The dynamics are
% \dot x = v
% \dot v = u
%
% where the control u aims to reach the target x
%
% Inputs: x - target state
%         visualize - whether to visualize the results
%
% Outputs: grids - two grid structures in a cell corresponding to datas
%          datas - 2D value functions in a cell
%          tau   - time vector corresponding to datas
%
% Mo Chen, 2015-08-25

% Default states and visualization option
if nargin<1
  x = [0 10 0 0]';
end

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 10;                    % End time.

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

%---------------------------------------------------------------------------
% Problem Parameters.
uMax = 3;
vRange = 10;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 71;

% Bounds on computational domain
g.min = [ x(1)-65 ; x(2)-2*vRange; x(3)-25 ; x(4)-vRange ];     
g.max = [ x(1)+25 ; x(2)+2; x(3)+25 ; x(4)+vRange ];  

g.dim = 4;                              % Number of dimensions

g.bdry = @addGhostExtrapolate;
g.N = zeros(4,1);
g.N(1) = 51;
for i = 2:g.dim
  g.N(i) = ceil(Nx/(g.max(1)-g.min(1))*(g.max(i)-g.min(i)));
end
g = processGrid(g);

TD_out.g = g;
TTR_out.g = g;

% ----------------- Target -----------------
TD_out.value = shapeRectangleByCorners(g, x-3*g.dx, x+3*g.dx);
TTR_out.value = 1e5 * ones(size(TD_out.value));
TTR_out.value(TD_out.value <= 0) = 0;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.uMax = uMax;

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

%--------------------------------------------------------------------------
accuracy = 'medium';

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

integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');

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
% Loop until tMax (subject to a little roundoff).
tNow = 0;
while(tMax - tNow > small * tMax)
  % How far to step?
  tSpan = [tNow, tMax];
  
  % Reshape data array into column vector for ode solver call.
  y0 = TD_out.value(:);

  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  TD_out.value = reshape(y, g.shape);
  tNow = t(end);
  
  TTR_out.value(TD_out.value < 0) = ...
    min(TTR_out.value(TD_out.value < 0), tNow);
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

checkStructureFields(schemeData, 'uMax', 'grid');

grid = schemeData.grid;
uMax = schemeData.uMax;

% Dynamics:
% \dot{x}_1 = v_1
% \dot{v}_1 = u_1

% quadrotor 1 minimizes value, quadrotor 2 maximizes value
hamValue = deriv{1} .* grid.xs{2} + ...
  (deriv{2}>=0) .* (deriv{2}) * (-uMax) + ...
  (deriv{2}<0) .* (deriv{2}) * uMax + ...
  deriv{3} .* grid.xs{4} + ...
  (deriv{4}>=0) .* (deriv{4}) * (-uMax) + ...
  (deriv{4}<0) .* (deriv{4}) * uMax;


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


checkStructureFields(schemeData, 'uMax', 'grid');

grid = schemeData.grid;
uMax = schemeData.uMax;

switch dim
  case 1
    alpha = abs(grid.xs{2});
    
  case 2
    alpha = uMax;
  
  case 3
    alpha = abs(grid.xs{4});
    
  case 4
    alpha = uMax;    
  otherwise
    error([ 'Partials only exist in dimensions 1-4' ]);
end
end