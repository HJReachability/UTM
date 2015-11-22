classdef Linpath < Node
  % linear path class
  
  properties
    % endpoints: z0, z1, always converted to 2x1 array
    z0
    z1
    
    % heading unit vector
    ds
    
    % function handle
    fn
    
    % travel speed
    speed
    
    h % plot handle
  end % end properties
    
  methods
    function obj = Linpath(z0, z1, speed)
      % function obj = highway(z0, z1, speed)
      % Constructor for linpath class
      %
      % Inputs:  z0     - starting point (in 2D)
      %          z1     - ending point
      %          speed  - speed of travel
      %
      % Output:  obj    - linpath object
      %
      % Mo Chen, 2015-07-21
      
      % Default speed
      if nargin<3
        speed = 10;
      end
      
      obj.speed = speed;
      
      % function handle representing highway
      if numel(z0) ~= 2 || numel(z1) ~= 2
        error('Starting and ending points must be in 2D!')
      end
      
      if ~iscolumn(z0);
        z0 = z0';
      end
      
      if ~iscolumn(z1)
        z1 = z1';
      end
      
      % Set the properties
      obj.z0 = z0;
      obj.z1 = z1;
      obj.fn = obj.generateFn;
      
      % compute highway heading
      obj.ds = zeros(2,1);
      for i = 1:2
        obj.ds(i) = z1(i) - z0(i);
      end
      obj.ds = obj.ds / norm(obj.ds);
    end % end constructor
  end % end methods
end % end class

