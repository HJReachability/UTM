function [ datax, datay, g, tau ] = quad2D_joinHighway(x, visualize)
% quad3D: demonstrate the 3D quadrotor collision avoidance example
%
%  [ data, g, data0 ] = quad3D(accuracy)
%
% The relative coordinate dynamics are
%
%   \dot x      = v_r = v1 - v2
%   \dot v_r    = u1 - u2
%	\dot v2     = u2
%
%   where,
%     input u2 trying to avoid the target
%	  input u1 trying to hit the target.
%
% Parameters:
%
%   accuracy     Controls the order of approximations.
%                  'low'         Use odeCFL1 and upwindFirstFirst.
%                  'medium'      Use odeCFL2 and upwindFirstENO2 (default).
%                  'high'        Use odeCFL3 and upwindFirstENO3.
%                  'veryHigh'    Use odeCFL3 and upwindFirstWENO5.
%
%   data         Implicit surface function at t_max.
%   g            Grid structure on which data was computed.
%   data0        Implicit surface function at t_0.

%---------------------------------------------------------------------------
% You will see many executable lines that are commented out.
%   These are included to show some of the options available; modify
%   the commenting to modify the behavior.

if nargin<2
    visualize = 1;
end

if nargin<1
    x = [0 0 0 0];
end

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 3.8;                    % End time.
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
uMax = 1.7;
uMin = -1.7;
vMax = 5;
vMin = -5;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 41;

% Create the grid.
g.dim = 2;                              % Number of dimensions
g.min = [ -10 ; 2.1*vMin ];     % Bounds on computational domain
g.max = [ 10 ; 2.1*vMax ];
g.bdry = @addGhostExtrapolate;
g.N = [ Nx; ceil(Nx/(g.max(1)-g.min(1))*(g.max(2)-g.min(2)))];
g = processGrid(g);
% g.dx
% g.N

% ----------------- Target -----------------
% Below minimum relative distance, small relative velocity, any velocity in x
datax = shapeRectangleByCorners(g, [x(1); x(2)]-1.1*g.dx, [x(1); x(2)]+1.1*g.dx);

% Below minimum relative distance, small relative velocity, any velocity in x
datay = shapeRectangleByCorners(g, [x(3); x(4)]-1.1*g.dx, [x(3); x(4)]+1.1*g.dx);

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.uMax = uMax;
schemeData.uMin = uMin;

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
    
    [~, h1] = contour(g.xs{1}, g.xs{2}, datax, [0 0],'r'); hold on
    contour(g.xs{1}, g.xs{2}, datax, [0 0],'r--');
    
    [~, h2] = contour(g.xs{1}, g.xs{2}, datay, [0 0],'b');
    contour(g.xs{1}, g.xs{2}, datay, [0 0],'b--');
    
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
    [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
        integratorOptions, schemeData);
    datax = cat(3, datax, reshape(y, g.shape));
    
    % Reshape data array into column vector for ode solver call.
    y0 = datay(:,:,end);
    y0 = y0(:);
    [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
        integratorOptions, schemeData);
    datay = cat(3, datay, reshape(y, g.shape));
    
    tNow = t(end);
    tau = cat(1, tau, tNow);
    
    % Create new visualization.
    if visualize
        h1.ZData = datax(:,:,end);
        h2.ZData = datay(:,:,end);
    end
    %
         drawnow;
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

checkStructureFields(schemeData, 'uMax','uMin');

grid = schemeData.grid;

uMax = schemeData.uMax;
uMin = schemeData.uMin;

% Dynamics:
% \dot{x} = \dot{v}
% \dot{v} = u

% quadrotor minimizes value
hamValue = deriv{1} .* grid.xs{2} + ...
    (deriv{2}>=0) .* (deriv{2}) * uMin + ...
    (deriv{2}<0) .* (deriv{2}) * uMax;

% backwards reachable set
hamValue = -hamValue;


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


checkStructureFields(schemeData, 'uMax', 'uMax');

grid = schemeData.grid;

uMax = schemeData.uMax;
uMin = schemeData.uMin;

switch dim
    case 1
        alpha = abs(grid.xs{2});
        
    case 2
        alpha = max(abs([uMin uMax]));
        
    otherwise
        error([ 'Partials only exist in dimensions 1-2' ]);
end
