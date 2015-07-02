classdef platoon < handle
    
    properties
        
        ID                      % ID of platoon 
        
        vehicle     % Handles to all vehicles in platoon
        
        hw  % handle to the highway the platoon is on
        
        n                       % Number of vehicles in platoon
        
        nmax                    % Maximum allowable number of vehicles in platoon
        
        vList             % List of slots in platoon (vector of length nmax)
                                % 1:  Slot is occupied
                                % 0:  Slot is empty
                                % -1: Slot is pending (a vehicle is
                                %     attempting to occupy the slot by joining
                                %     platoon
        
        vJoin             % Cell array of handles to vehicles attempting to join platoon
                                % -1:   No vehicle attempting to join this
                                %       slot
                                % otherwise: handle to vehicle attempting
                                %       to join this slot
        
        followTime              % Separation time for followers
        
        FP                      % Front platoon
        BP                      % Back platoon
        
        % Footprint of platoon
        safeV
    end
    
    methods
        function obj = platoon(leader, hw, nmax,followTime)
            % function obj = platoon(leader,hw,nmax,followTime)
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
            obj.vList = zeros(nmax,1);
            obj.vList(1) = 1;
            
            obj.vJoin = {};
            
            obj.hw = hw; % Point to highway
            obj.hw.ps = [obj.hw.ps obj]; % make highway object point to this platoon
            
            % Make sure leader is within the width of the highway
            [~, dist] = obj.hw.highwayPos(obj.vehicle.x(obj.vehicle.pdim));
            if dist > obj.hw.width
                error('Vehicle is too far to be added to platoon on this highway!')
            end

            % Set vehicle mode to leader
            obj.vehicle.q = 'Leader';
            obj.vehicle.idx = 1;
            obj.vehicle.p = obj;
            obj.vehicle.Leader = obj.vehicle;
            
            % Set BQ and FQ to itself for leader
            obj.vehicle.FQ = leader;
            obj.vehicle.BQ = leader;
            
            % Initialize platoon pointers to self
            obj.FP = obj;
            obj.BP = obj;
            
            % Remove merge highway value function
            obj.vehicle.mergeHighwayV = [];
            
        end
                
%         function annex(obj,platoon) % Append trailing platoon at the back of obj
%             % UNUSED?? Should put this in a separate file
%             
%             if platoon.FP ~= obj
%                 warning([
%                     sprintf('Cannot append platoon.\n'),...
%                     sprintf('\tPlatoon %s is behind platoon %s.\n',platoon.ID, platoon.FQ.ID),...
%                     sprintf('\tCan only append platoons directly behind current platoon.\n')
%                 ]);
%             end
%             
%             % Update platoon pointers
%             if platoon.BP == platoon
%                 obj.BP          = obj;
%             else
%                 obj.BP          = platoon.BP;
%                 platoon.BP.FP   = obj;
%             end
%             
%             % Update vehicle pointers
%             obj.vehicle{obj.n}.BQ   = platoon.vehicle{1};
%             platoon.vehicle{1}.FQ   = obj.vehicle{obj.n};
%             
%             % Update follower vehicles in old trailing platoon
%             for i=1:platoon.n
%                 platoon.vehicle{i}.platoon = obj;
%                 platoon.vehicle{i}.idx = obj.n + platoon.vehicle{i}.idx;
%             end
%             % Concatenate all lists and update info
%             obj.n         =  obj.n     +    platoon.n;
%             obj.vehicle   = [obj.vehicle    platoon.vehicle];
%             obj.IDvehicle = [obj.IDvehicle  platoon.IDvehicle];
%             % Delete old platoon object
%             delete(platoon)
%         end
        
        
        
    end
end