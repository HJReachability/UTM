classdef Plane < Vehicle
  % Note: Since plane is a "handle class", we can pass on
  % handles/pointers to other plane objects
  % e.g. a.platoon.leader = b (passes b by reference, does not create a copy)
  % Also see constructor
  
  properties
    % Angular control bounds (only applicable if using TFM)
    wMin = -2*pi/10
    wMax = 2*pi/10
    
    % speed control bounds (only applicable if using TFM)
    vMax
    vMin
    
    % state dimension is set to 3 and number of controls is set to 1 if 
    % Plane is initialized with a 3D state
    nx = 4;
    nu = 2;
    
    % speed (if 3D)
    speed = 10
  end
  
  properties(Constant)
    % distance from max/min x or y to edge of plot
    WIND = 5
    DEFAULT_V = 10;
  end
  
  methods
    function obj = Plane(x)
      % obj = plane(x)
      %
      % Constructor. Creates a plane object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{x}_1 = v * cos(x_3)
      %    \dot{x}_2 = v * sin(x_3)
      %    \dot{x}_3 = u
      %         uMin <= u <= uMax
      %
      % Inputs:   x         - state: [xpos; ypos; theta; v]
      %
      % Output:   obj       - a Plane object
      %
      % Mahesh Vashishtha, 2015-10-26
      % Modified, Mo Chen, 2015-11-04
      
      if numel(x) ~= 3
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end

      obj.nx = 3;
      obj.nu = 1;

      obj.x = x;
      obj.xhist = obj.x;
    end
    
    %%
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

  end % end methods
end % end classdef
