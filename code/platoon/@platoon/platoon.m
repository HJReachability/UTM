classdef platoon < handle
    
    properties
        
        ID                      % ID of platoon 
        
        vehicle     % Handles to all vehicles in platoon
        
        hw  % handle to the highway the platoon is on
        
        n                       % Number of vehicles in platoon
        
        nmax                    % Maximum allowable number of vehicles in platoon
        
        followTime              % Separation time for followers
        
        FP                      % Front platoon
        BP                      % Back platoon
        
        % Footprint of platoon
        safeV
    end
    
    methods
        function obj = platoon(leader, hw, nmax,followTime)
            % function obj = platoon(leader,nmax,followTime)
            % Constructor for platoon object
            %
            % Must specify a leader and a highway
            %
            
            if nargin<3, nmax = 5; end
			obj.nmax = nmax;
			
            if nargin<4, followTime = 2; end
            obj.followTime = followTime;
			
            % Preliminary
            obj.ID = leader.ID; 
            obj.n = 1;
			
            obj.vehicle = leader;
            
            obj.hw = hw;
            
            % Set FQ to itself for leader
            leader.FQ = leader;
			
            % Initialize platoon pointers to self
            obj.FP = obj;
            obj.BP = obj;
            
        end
                
        function annex(obj,platoon) % Append trailing platoon at the back of obj
            % UNUSED?? Should put this in a separate file
            
            if platoon.FP ~= obj
                warning([
                    sprintf('Cannot append platoon.\n'),...
                    sprintf('\tPlatoon %s is behind platoon %s.\n',platoon.ID, platoon.FQ.ID),...
                    sprintf('\tCan only append platoons directly behind current platoon.\n')
                ]);
            end
            
            % Update platoon pointers
            if platoon.BP == platoon
                obj.BP          = obj;
            else
                obj.BP          = platoon.BP;
                platoon.BP.FP   = obj;
            end
            
            % Update vehicle pointers
            obj.vehicle{obj.n}.BQ   = platoon.vehicle{1};
            platoon.vehicle{1}.FQ   = obj.vehicle{obj.n};
            
            % Update follower vehicles in old trailing platoon
            for i=1:platoon.n
                platoon.vehicle{i}.platoon = obj;
                platoon.vehicle{i}.idx = obj.n + platoon.vehicle{i}.idx;
            end
            % Concatenate all lists and update info
            obj.n         =  obj.n     +    platoon.n;
            obj.vehicle   = [obj.vehicle    platoon.vehicle];
            obj.IDvehicle = [obj.IDvehicle  platoon.IDvehicle];
            % Delete old platoon object
            delete(platoon)
        end
        
        
        
    end
end