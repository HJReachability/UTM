function [ data, g, tau ] = platoon3DcollisionY (accuracy)
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

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 3.6;                    % End time.
plotSteps = 4;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
singleStep = 1;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

% V stores value function at different time points
V = cell(1,plotSteps);

%---------------------------------------------------------------------------
% Problem Parameters.
% States: x_r, v_r, v_1 (changed from v_2 to v_1 by Mo)
%   d           Safe separation distance.
%   u1Max       Maximum acceleration of quadrotor1.
%   u1Min       Minimum acceleration of quadrotor1.
%   u2Max       Maximum acceleration of quadrotor2.
%   u2Min       Minimum acceleration of quadrotor2.  
%   v1Max       Maximum velocity of quadrotor1.
%   v1Min       Minimum velocity of quadrotor1.
%   v2Max       Maximum velocity of quadrotor2.
%   v2Min       Minimum velocity of quadrotor2.
% quadrotor 1 is the evader, quadrotor 2 is the pursuer
d = 2;
u1Max = 1.7;
u1Min = -1.7;
u2Max = 1.7;
u2Min = -1.7;
v1Max = 5;
v1Min = -5;
v2Max = 5;
v2Min = -5;
vrMax = v1Max-v2Min;
vrMin = v1Min-v2Max;

%---------------------------------------------------------------------------
% What level set should we view?
level = 0;

% Visualize the 2D reachable set.
displayType = 'surface';

% Pause after each plot?
pauseAfterPlot = 0;

% Delete previous plot before showing next?
deleteLastPlot = 1;

% Visualize the angular dimension a little bigger.
% aspectRatio = [ 1 1 0.4 ];

% Plot in separate subplots (set deleteLastPlot = 0 in this case)?
useSubplots = 0;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 101;
Nv = 51;
% Create the grid.
spacing=6*sqrt(2);
g.dim = 3;                              % Number of dimensions
g.min = [ -40-3*spacing ; 1.1*vrMin; 1.3*v1Min ];     % Bounds on computational domain
g.max = [ 40 ; 1.1*vrMax; 1.3*v1Max ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
g.N = [ round(Nx*(g.max(1)-g.min(1))/(80)); Nv; ceil(Nv/(g.max(2)-g.min(2))*(g.max(3)-g.min(3))) ];
g = processGrid(g);
g.dx
g.N

% ----------------- Unsafe conditions -----------------
% Below minimum relative distance, any relative velocity, any velocity
data = shapeRectangleByCorners(g, [-d; -inf; -inf], [d; inf; inf]);


% data = dataMS;

% ----- "Win" conditions (unsafe conditions for other vehicle) -----
% Need this if we do not allow vehicle 2 to go above speed limit

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.u1Max = u1Max;
schemeData.u1Min = u1Min;
schemeData.u2Max = u2Max;
schemeData.u2Min = u2Min;
% schemeData.v1Max = v1Max;
% schemeData.v1Min = v1Min;
% schemeData.v2Max = v2Max;
% schemeData.v2Min = v2Min;
% schemeData.vrMax = vrMax;
% schemeData.vrMin = vrMin;

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
  accuracy = 'veryHigh';
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

% %---------------------------------------------------------------------------
% % Restrict the Hamiltonian so that reachable set only grows.
% %   The Lax-Friedrichs approximation scheme MUST already be completely set up.
% innerFunc = schemeFunc;
% innerData = schemeData;
% clear schemeFunc schemeData;
% 
% % Wrap the true Hamiltonian inside the term approximation restriction routine.
% schemeFunc = @termRestrictUpdate;
% schemeData.innerFunc = innerFunc;
% schemeData.innerData = innerData;
% schemeData.positive = 0;

% % Set up masking so that the reachable set does not propagate through the
% % avoid set
% schemeData.maskData = -avoid(:);
% schemeData.maskFunc = @max;
% 
% % Let the integrator know what function to call.
% integratorOptions = odeCFLset(integratorOptions, 'postTimestep', @postTimestepMask);

%---------------------------------------------------------------------------
% Initialize Display
f = figure;

% Set up subplot parameters if necessary.
if(useSubplots)
  rows = ceil(sqrt(plotSteps));
  cols = ceil(plotSteps / rows);
  plotNum = 1;
  subplot(rows, cols, plotNum);
end

% h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);
% return
v1slice = 1.05*v1Max;
[g2D, data2D] = proj2D(g, [0 0 1], 101, data, v1slice);
[~, h1] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0],'r');
contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0],'r--');

[g2D, data2D] = proj2D(g, [0 0 1], 101, data, 0);
[~, h2] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0],'b');
contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0],'b--');

xlabel('x_r')
ylabel('v_r')
% zlabel('v_1')

% camlight right;  camlight left;
hold on;
axis(g.axis);
% daspect(aspectRatio);
drawnow;
% return
%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
tau = tNow;
while(tMax - tNow > small * tMax)
  % Reshape data array into column vector for ode solver call.
  y0 = data(:,:,:,end);
  y0 = y0(:);
  
  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tPlot) ];
  
  % Take a timestep.
  [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
                  integratorOptions, schemeData);
  tNow = t(end);
  tau = cat(1, tau, tNow);
  % Get back the correctly shaped data array
  data = cat(4, data, reshape(y, g.shape));
  
  if(pauseAfterPlot)
    % Wait for last plot to be digested.
    pause;
  end

  % Delete last visualization if necessary.
  if(deleteLastPlot)
    delete(h1);
    delete(h2);
  end

  % Move to next subplot if necessary.
  if(useSubplots)
    plotNum = plotNum + 1;
    subplot(rows, cols, plotNum);
  end

  % Create new visualization.
%   h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);
    [g2D, data2D] = proj2D(g, [0 0 1], 101, data(:,:,:,end), v1slice);
    [~, h1] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0],'r');
    
    [g2D, data2D] = proj2D(g, [0 0 1], 101, data(:,:,:,end), 0);
    [~, h2] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0],'b');    
    drawnow
end

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);
% save('VSingleAxis.mat', 'V', 'g')

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

checkStructureFields(schemeData, 'u1Max','u1Min','u2Max','u2Min');

grid = schemeData.grid;

u1Max = schemeData.u1Max;
u1Min = schemeData.u1Min;
u2Max = schemeData.u2Max;
u2Min = schemeData.u2Min;

% Dynamics:
% \dot{x}_r = \dot{v}_r
% \dot{v}_r = u_1 - u_2
% \dot{v}_1 = u_1

% quadrotor 1 maximizes value, quadrotor 2 minimizes value
hamValue = deriv{1} .* grid.xs{2} + ...
    (deriv{2}+deriv{3}>=0) .* (deriv{2}+deriv{3}) * u1Max + ...
    (deriv{2}+deriv{3}<0) .* (deriv{2}+deriv{3}) * u1Min + ...
    (-deriv{2}>0) .* (-deriv{2}) * u2Min + ...
    (-deriv{2}<=0) .* (-deriv{2}) * u2Max;


% backwards reachable set
hamValue = -hamValue;
    

% In general, avoid nested for loops in MATLAB! - Mo
% We can have an avoid set outside fo the hamiltonian
%
% hamValue = zeros(grid.N(1),grid.N(2),grid.N(3));
% 
% for i = 1:1:grid.N(1)
%     for j = 1:1:grid.N(2)
%         for k = 1:1:grid.N(3)
%             
%             v2 = grid.xs{3}(i,j,k);
%             v1 = grid.xs{2}(i,j,k) + grid.xs{3}(i,j,k);
%             
%             if v1>schemeData.v1Min & v1<schemeData.v1Max
%                 u1 = (schemeData.u1Min+schemeData.u1Max)/2 + sign(deriv{2}(i,j,k)) * (schemeData.u1Min - schemeData.u1Max)/2;
%             else
%                 u1 = 0;
%             end
%             
%             if v2>schemeData.v2Min & v2<schemeData.v2Max
%                 u2 = (schemeData.u2Min+schemeData.u2Max)/2 + sign(deriv{3}(i,j,k)-deriv{2}(i,j,k)) * (schemeData.u2Max - schemeData.u2Min)/2;
%             else
%                 u2 = 0;
%             end
%             
%             hamValue(i,j,k) = -(deriv{1}(i,j,k) * grid.xs{2}(i,j,k) ...
%                 + deriv{2}(i,j,k) * u1 ...
%                 + (deriv{3}(i,j,k) - deriv{2}(i,j,k)) * u2);
%         end
%     end
% end
         

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


checkStructureFields(schemeData, 'u1Max', 'u2Max', 'u1Min', 'u2Min');

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

    case 3
        alpha = max(abs([u2Min u2Max]));
        
    otherwise
        error([ 'Partials for only exist in dimensions 1-3' ]);
end
