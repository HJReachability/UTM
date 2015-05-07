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
            if nargin<3
                speed = 3;
            end
            
            if nargin<4
                width = 6;
            end
            
            % function handle representing highway
            obj.fn = generateFn(z0, z1);
            
            % compute highway heading
            obj.ds = zero(2,1);
            for i = 1:2
                obj.ds(i) = z1(i) - z0(i);
            end
            obj.ds = obj.ds / norm(obj.ds);
            
            warning('Need to specify connections property')
        end
        
        function fn = generateFn(z0, z1)
            x0 = z0(1);
            y0 = z0(2);
            
            x1 = z1(1);
            y1 = z1(2);
            
            fn = @(s) [(1-s)*x0 + s*x1; (1-s)*y0 + s*y1];
        end
        
        function s = highwayPos(z)
            % Returns position on highway given absolute position
            % (orthogonal projection)
            %
            % Returns -1 if given position is outside of the width of the
            % highway
        end
    end
    
end

