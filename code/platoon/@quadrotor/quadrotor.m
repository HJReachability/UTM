classdef quadrotor < vehicle
  % Note: Since quadrotor is a "handle class", we can pass on
  % handles/pointers to other quadrotor objects
  % e.g. a.platoon.leader = b (passes b by reference, does not create a copy)
  % Also see constructor
  
  properties
    uMin        % Control bounds
    uMax
    
    vMax        % Speed bounds
    vMin
  end % end properties
 
  properties(Constant)
    % Dimensions of state and control
    nx = 4;
    nu = 2;
    
    % A and B matrices (consider putting these as constant properties)
    A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
    B = [0 0; 1 0; 0 0; 0 1];
      
    % Indices of position and velocity variables
    pdim = [1 3];
    vdim = [2 4];

    tauInt = 1.5;     % Separation distance with vehicles inside platoon (depends on mode)
    tauExt = 3;      % Separation distance with objects outside platoon    
  end % end properties(Constant)
  
  methods
    function obj = quadrotor(ID, x, reachInfo)
      % obj = quadrotor(ID, dt, x, reachInfo)
      %
      % Constructor. Creates a quadrotor object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{p}_x = v_x
      %    \dot{v}_x = u_x
      %    \dot{p}_y = v_y
      %    \dot{v}_y = u_y
      %       uMin <= u_x <= uMax
      %
      % Inputs:   ID        - unique ID of the quadrotor
      %           x         - state: [xpos; xvel; ypos; yvel]
      %           reachInfo - reachable set information
      %                    .uMax, .uMin - max and min inputs
      %                    .vMax, .vMin - max and min velocities
      %
      % Output:   obj       - a quadrotor object
      %
      % Mo Chen, Qie Hu, 2015-05-22
      % Modified by Mo Chen, 2015-07-06
      
      % Preliminary
      obj.ID = ID;

      % Initial state and state history
      if nargin < 2
        obj.x = zeros(obj.nx, 1);
      else
        obj.x = x;
      end
      obj.xhist = obj.x;
      
      % Control bounds and Reachable set
      if nargin<3
        obj.uMax = 1.7;
        obj.uMin = -1.7;
        obj.vMax = 5;
        obj.vMin = -5;
      else
        obj.uMax = reachInfo.uMax;
        obj.uMin = reachInfo.uMin;
        obj.vMax = reachInfo.vMax;
        obj.vMin = reachInfo.vMin;
      end
      
    end % end constructor
  end % end methods
end % end class