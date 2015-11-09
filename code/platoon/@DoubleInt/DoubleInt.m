classdef DoubleInt < Vehicle
  properties
    uMin = -3    % Control bounds
    uMax = 3
    
    h_state
  end % end properties
  
  properties(Constant)
    % Dimensions of state and control
    nx = 2;
    nu = 1;
    
  end % end properties(Constant)
  
  methods
    function obj = DoubleInt(x)
      
      % Make sure initial state is 4D
      if numel(x) ~= 2
        error('DoubleInt state must be 2D.')
      end

      % Make sure initial state is a column vector
      if ~iscolumn(x)
        x = x';
      end
      
      obj.x = x;
      obj.xhist = x;
      
    end % end constructor
  end % end methods
end % end class