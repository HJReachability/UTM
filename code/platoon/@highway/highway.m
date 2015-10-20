classdef highway < linpath
  % Highway class (inherits linpath class)
  
  properties
    % width of highway
    width
    
    % liveness value function for steering quadrotors to points on highway
    liveV 

    % highway connections/segways
    connections
    % CURRENTLY:
    % n by 1 vector where the ith element is the s parameter on the current
    % highway where it intersects with the ith highway
    %
    % ALTERNATIVE OPTION FOR THE FUTURE:
    % n by 2 cell structure
    % {s0, i/o, {hw0, s_onhw0};
    %  s1, i/o, {hw1, s_onhw1};
    %  ... }
    
    ps % platoons on the highway
  end
  
  methods
    function obj = highway(z0, z1, speed, fourD, width)
      % function obj = highway(z0, z1, speed, width)
      % Constructor for highway class
      %
      % Inputs:  z0     - starting point (in 2D)
      %          z1     - ending point
      %          speed  - speed of travel
      %          width  - width of highway
      %
      % Output:  obj    - highway object
      %
      % Mo Chen, 2015-05-25
      % Modified: 2015-07-21

      %% Default speed and width
      if nargin<3
        speed = 3;
      end
      
      % Call constructor of the superclass linpath
      obj@linpath(z0, z1, speed);
      
      if nargin<5
        obj.width = 6;
      else
        obj.width = width;
      end      
      
      %% Whether to use 4D reachable set
      if nargin<4
        fourD=1;
      end
     
      %% Compute liveness reachable set for joining the highway
      % highway direction and speed
      velocity = obj.ds * obj.speed;
      
      % Compute 2D value function that could be used to reconstruct the 4D
      % value function
      [grids, datas, tau] = quad2D_liveness( ...
                                         [0 velocity(1) 0 velocity(2)], 0);
      
      % If 4D reachable set is specified, then reconstruct it; otherwise,
      % simply store the 2D reachable sets and reconstruct later
      if fourD
        gridLim = ...
          [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
        [~, ~, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));
        
        obj.liveV.g = TTR_out.g;
        obj.liveV.data = TTR_out.value;
        obj.liveV.grad = TTR_out.grad;
        obj.liveV.tau = tau;
        
      else
        obj.liveV.g = grids;
        obj.liveV.data = datas;
        obj.liveV.tau = tau;
        obj.liveV.grad = [];
      end
      
      warning('Need to specify connections property')
    end
  end
end

