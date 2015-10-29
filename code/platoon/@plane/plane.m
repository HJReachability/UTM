classdef plane < vehicle
  % Note: Since plane is a "handle class", we can pass on
  % handles/pointers to other plane objects
  % e.g. a.platoon.leader = b (passes b by reference, does not create a copy)
  % Also see constructor  
  
  properties    
    wMin        % angular control bounds
    wMax
    
    vMax        % speed control bounds
    vMin    

  end
  
  properties(Constant)
    % dimensions of state and control
    nx = 4;
    nu = 2;
    
    % distance from max/min x or y to edge of plot
    WIND = 5
    DEFAULT_V = 5;
    V_TOL = 2; % min. safe value of value function
  end
  
  methods
    function obj = plane(ID, x, reachInfo)
      % obj = plane(ID, x, reachInfo)
      %
      % Constructor. Creates a plane object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{x}_1 = v_x = x_4 * cos(x_3)
      %    \dot{x}_2 = v_y = x_4 * sin(x_3)
      %    \dot{x}_3 = u_1 = u_1
      %    \dot{x}_4 = u_2 = u_2
      %         uMin <= u <= uMax
      %
      % Inputs:   ID        - unique ID of the plane
      %           x         - state: [xpos; ypos; theta; v]
      %           reachInfo - reachable set information
      %                    .uMax, .uMin - max and min angular velocity
      %                    .vMax - max speed (positive)
      %
      % Output:   obj       - a quadrotor object
      %
      % Mahesh Vashishtha, 2015-10-26
      obj.ID = ID;  

      % Initial state and state history
      if nargin < 3
        obj.x = zeros(obj.nx, 1);
        obj.x(4) = plane.DEFAULT_V;
      else        
        if numel(x) ~= obj.nx
          error('Initial state does not have right dimension');
        end
        obj.x = x;
      end
      obj.xhist = obj.x;
      obj.u = [0 0];

      % Control bounds and Reachable set
      if nargin<4
        obj.wMax = 1;
        obj.wMin = -1;
        obj.vMax = 1;
        obj.vMin = -1;
      else
        obj.wMax = reachInfo.uMax;
        obj.wMin = reachInfo.uMin;
        obj.vMax = reachInfo.vMax;
        obj.vMin = reachInfo.vMin;
      end      
    end
 
    function x1 = computeState(obj, u, x0)      
      % function x1 = computeState(obj, u, x0)
      % Computes (DOES NOT update!) state based on control; use updateState to
      % update the state
      %
      % Inputs:   obj - current plane object
      %           u   - control (defaults to previous control)
      %           x0  - initial state (defaults to current state)
      %
      % Outputs:  x1  - final state
      %
      % Mahesh Vashishtha, 2015-10-26

      % If no control is specified, use previous control      
      if nargin  < 2
        u = obj.u;
      end
      if nargin < 3
        x0 = obj.x;
      end      
      % forwards euler (unstable for large dt)
      x1 = x0 + obj.dt * [x0(4)*cos(x0(3)); x0(4)*sin(x0(3)); u(1); u(2)];
    end
    
    function x1 = updateState(obj, u, x0)
      % function x1 = updateState(obj, u, x0)
      % Updates state based on control
      %
      % Inputs:   obj - current plane object
      %           u   - control (defaults to previous control)
      %           x0  - initial state (defaults to current state)
      %
      % Outputs:  x1  - final state
      %
      % Mahesh Vashishtha, 2015-10-26

      % If no control is specified, use previous control      
      if nargin < 2
        u = obj.u;
      else
        if numel(u) ~= plane.nu
          error('Control is wrong dimension')
        elseif any(cat(2,u < [obj.wMin obj.vMin], u > [obj.wMax obj.vMax]))
          error('Control not in bounds.')
        end
      end
    
      % If no state is specified, use current state
      if nargin < 3
        x0 = obj.x;
      end      
      x1 = obj.computeState(u, x0);

      % Update the state, state history, control, and control history
      obj.x = x1;
      obj.u = u;

      obj.xhist = cat(2, obj.xhist, obj.x);
      obj.uhist = cat(2, obj.uhist, obj.u);
    end
        
    function pos = getPosition(obj)
      % function pos = getPosition(obj)
      % Finds the position (x,y) of a plane
      pos = obj.x(1:2);
    end
    
    function vel = getVelocity(obj)
      % function vel = getPosition(obj)
      % Finds the velocity (v_x,v_y) of a plane      
      vel = obj.x(4) * [cos(obj.x(3)) sin(obj.x(3))];
    end  
    
    function relStates = getRelativeStates(obj, others)
      % function relStates = getRelativeStates(obj, others)
      % Finds state of other plane objects with respect to one plane
      %
      % Inputs:   obj - current plane object
      %           others - other planes whose relative states relative 
      %                    to obj are to be found; this
      %                    should be a n x 1 or 1 x n cell. 
      %
      % Outputs:  relStates - relative state of each other plane
      %
      % Mahesh Vashishtha, 2015-10-27
      others = checkVehiclesList(others, 'plane');      
      relStates = zeros(length(others),4);
      xp = obj.x;
      for i=1:length(others)        
        xe = others{i}.x;
        if size(xe, 1) ~= 4
          xe = xe';
        end
        if size(xp, 1) ~= 4
          xp = xp';
        end   
        xr = xp -xe;        
        % rotate position vector so it is correct relative to xe heading
        xr(1:2) = [cos(-xe(3)) -sin(-xe(3)); sin(-xe(3)) cos(-xe(3))] * xr(1:2);
        xr = xr';
        if xr(3) >= 2*pi
          xr(3) = xr(3) - 2*pi;
        end
        if xr(3) < 0
          xr(3) = xr(3) + 2*pi;
        end
        vel_comps = arrayfun(@getVelocity, [obj; others{i}],'un',0);
        xr(4) = norm(cell2mat(vel_comps(1,:)) - cell2mat(vel_comps(2,:)));
        relStates(i,:) = xr;        
      end        
    end    
    
    function collided = isCollided(obj, others, radius)
      % function collided = isCollided(obj, others,radius)
      % Check if any other planes are within distance "radius" of this
      % plane
      %
      % Inputs:   obj - current plane object
      %           others - other planes whose positions should be checked
      %                    for collision; this should be a n x 1 or 1 x n 
      %                    cell. All vehicles need to be of the same type
      %                    so that a single safeV can be used
      %           radius - positive number. iff distance between planes is 
      %                    <= radius, a collision has occured
      % 
      %
      % Outputs:  collided - true iff distance between obj and any plane in
      %                     others is <= radius
      %
      % TODO: see if there is more efficient way to check each collision pair
      % Mahesh Vashishtha, 2015-10-27      
      others = checkVehiclesList(others, 'plane');
      relStates = obj.getRelativeStates(others);
      relPos = relStates(:,1:2);   
      collided = false;
      for i = 1:length(others)
        collided = or(all(abs(relPos(i,1:2)) <= [radius radius]), collided);
      end      
    end
       
    function [safe, uSafe, valuex] = isSafe(obj, others, safeV, costates) 
      % [safe, uSafe, valuex] = isSafe(obj, others, safeV, costates) 
      % Check if this plane is safe relative to other planes. Assume that 
      % all velocities are fixed and equal and that safeV is computed
      % assuming the same fixed velocity. Also returns optimal angular 
      % control input (this input is the optimal input for avoiding each 
      % plane in isolation)
      %
      % Inputs:  obj    - this plane
      %          other - other planes with whom safety should be checked
      %          safeV  - safety reachable set value function
      %
      % Outputs: safe   - boolean array indicating safety for each vehicle
      %          uSafe  - the optimal safe controllers
      %          valuex - the values of levelset function   
      %
      % Mahesh Vashishtha, 2015-10-27
      others = checkVehiclesList(others, 'plane');

      g = safeV.g;
      data = safeV.data;
      for i = 1:length(others)
        z = obj.getRelativeStates(others{i});
        z = z(1,:);
        z = z(1:3);
        valuex(i) = eval_u(g, data, z);      
        p = calculateCostate(g, costates, z);
        if p(1) * z(2) - p(2) * z(1) - p(3) > 0
          uSafe(i) = 1;
        else
          uSafe(i) = -1;
        end          
        safe(i) = isnan(valuex) || valuex >= obj.V_TOL;    
      end
    end
        
    function plotSafeV(obj, others, safeV, t)     
      % function plotSafeV(obj, others, safeV, t)
      %
      % Plots the safe region around others that obj must stay out of in order to
      % be safe.
      %
      % Inputs: obj   - this plane
      %         other - other plane
      %         safeV - Reachable set
      %         t     - time horizon
      %
      % TODO: implement this function
      % Mahesh Vashishtha, 2015-10-27  
      others = checkVehiclesList(others, 'plane'); 
    end          
  end   
  methods(Static)    
    function plotPositions(planes, axs) 
      % function plotSafeV(obj, others, safeV, t)
      %
      % Plots positions of a list of planes
      %
      % Inputs: planes - list of planes whose positions must be plotted
      %
      % Mahesh Vashishtha, 2015-10-27        
      planes = checkVehiclesList(planes, 'plane');
      clf
      for i = 1:length(planes)
          x = planes{i}.x;
          q  = quiver(x(1), x(2), x(4)*cos(x(3)), x(4)*sin(x(3)));
          hold on;
      end
      set(q, 'AutoScale', 'off');       
      axis(axs);
    end      
  end
end