classdef Quadrotor12D < Vehicle
  properties
    % Control bounds
    uMin
    uMax
    
    % Turn rate and speed are both controls; however, if vrange is a
    % scalar, then the Plane has constant speed
    nx = 12;
    nu = 4;
    
    % Position, velocity, dimensions
    pdim = 1:3;
    vdim = 4:6;
  end
  
  methods
    function obj = 12Dquadrotor(pos, uMin, uMax,aux_states)
      % obj = 12Dquadrotor(pos, uMin, uMax, aux_states)
      %
      % Constructor. Creates a 12Dquadrotor object
      %
      % Dynamics:
      %     \dot x_1  = x_4
      %     \dot x_2  = x_5
      %     \dot x_3  = x_6
      %     \dot x_4  = -(\cos x_7 \sin x_8 \cos x_9 + \sin x_7 \sin x_9) u_1/m
      %     \dot x_5  = -(\cos x_7 \sin x_8 \sin x_9 - \sin x_7 \cos x_9) u_1/m
      %     \dot x_6  = g - (\cos x_7 \cos x_8) u_1/m
      %     \dot x_7  = x_10 + \sin x_7 \tan(x_8) x_11 + \cos x_7 \tan(x_8) x_12
      %     \dot x_8  = \cos x_7 x_11 - \sin x_7 x_12
      %     \dot x_9  = (\sin x_7/\cos x_8)*x_11 + (\cos x_7/\cos x_8) x_12
      %     \dot x_10 = ...
      %
      % Inputs:
      %   x      - state: [xpos; ypos; theta]
      %   wMax   - maximum turn rate
      %   vrange - speed range
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a Plane object
      
      if numel(x) ~= 3
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(pos)
        pos = pos';
      end

      
      obj.x = x;
      obj.xhist = obj.x;
    end
    
end % end classdef
