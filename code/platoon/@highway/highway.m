classdef highway < handle
    %HIGHWAY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % endpoints: z0, z1
        z0
        z1
        
        % heading
        ds
        
        % function handle
        fn
        
        % highway connections/segways
        connections % n by 2 cell structure
        % {s0, i/o, {hw0, s_onhw0};
        %  s1, i/o, {hw1, s_onhw1}; ...
        %                      }
        
        speed % highway travel speed
        
        width
    end
    
    
    methods
        function obj = highway(z0, z1, speed, width)
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
            
            % Default speed and width
            if nargin<3, obj.speed = 3; 
            else         obj.speed = speed;
            end
            
            if nargin<4, obj.width = 6; 
            else         obj.width = width;
            end
            
            % function handle representing highway
            obj.z0 = z0;
            obj.z1 = z1;
            obj.fn = obj.generateFn;
            
            % compute highway heading
            obj.ds = zeros(2,1);
            for i = 1:2, obj.ds(i) = z1(i) - z0(i); end
            obj.ds = obj.ds / norm(obj.ds);
            
            warning('Need to specify connections property')
        end
        
        function fn = generateFn(obj)
            % function fn = generateFn(z0, z1)
            %
            % Computes the function handle that describes the highway
            %
            % Inputs:  z0, z1 - starting and ending points
            %
            % Output:  fn     - function handle for the highway
            %             fn(s) specifies a point on the highway
            %                fn(0) = z0, fn(1) = z1
            %                fn(s), 0<s<1 picks a point between z0 and z1, with
            %                             linear spacing
            %
            % Mo Chen, 2015-05-25
            z0 = obj.z0;
            z1 = obj.z1;
            fn = @(s) [(1-s)*z0(1) + s*z1(1); (1-s)*z0(2) + s*z1(2)];
        end
        
    end
    
end

