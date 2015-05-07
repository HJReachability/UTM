classdef quadrotor < handle
    
% Note: Since quadrotor is a "handle class", we can pass on
% handles/pointers to other quadrotor objects
% e.g. a.platoon.leader = b (passes b by reference, does not create a copy)

    properties
        ID          % ID number (global, unique)
        
        tau         % Separation distance (depends on mode)
        
        x           % State (4D)
        u           % Recent control signal
        
        xhist       % History of state
        uhist       % History of control
        
        dt          % Sampling time
        
        nx          % Dimension of state
        nu          % Dimension of control
        
        umin        % Control bounds
        umax
        
        vMax        % speed bounds
        vMin        
        
        q           % Mode
                    %   1: Free
                    %   2: Follower
                    %   3: Leader
                    %   4: Faulty
                    %   5: EmergLeader
                    
        A           % Dynamics
        B
        
        msg         % Cell array of structures for V2V/V2I messages
        idx         % Vehicle index in platoon
        
        platoon     % Structure containing platoon information
        platoonHeading     % Only available for Leader
         
        % Footprint from reachable set
        safeV       
        
        % Travel route
        rpath
    end
    
    methods
        function obj = quadrotor(ID, dt, x, rpath, reachInfo)
            % Constructor
            
            % Preliminary
            obj.ID = ID;
            obj.dt = dt;
            obj.q = 'Free';
            
            % Control bounds
            if nargin<5
                obj.umax = 1;
                obj.umin = -1;                
            else
                obj.umax = reachInfo.uMax;
                obj.umin = reachInfo.uMin;
            end
            
            obj.vMax = reachInfo.vMax;
            obj.vMin = reachInfo.vMin;
            
            % A matrix
            obj.A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
            
            % B matrix
            obj.B = [0 0; 1 0; 0 0; 0 1];
                        
            % Dimensions
            [obj.nx, obj.nu] = size(obj.B);
            
            % Initial state
            if nargin < 3
                obj.x = zeros(obj.nx, 1);
            else
                obj.x = x;
            end
            obj.xhist = obj.x;
            
            % Initial control
            obj.u = zeros(obj.nu, 0);
            obj.uhist = obj.u;
            
            % Reachable set
            if nargin<5
                obj.safeV = [];
            else
                obj.safeV = reachInfo.safeV;
            end
            
            if nargin<4
                obj.rpath = [];
            else
                obj.rpath = rpath;
            end
            
            % Handle to platoon and index if appropriate
            obj.platoon = []
            obj.idx = 0;
        end
        
            
        function p = newPlatoon(obj,nmax)
            % Check if vehicle is Free
            if ~strcmp(obj.q,'Free')
                warning([
                    sprintf('Cannot create new platoon.\n'),...
                    sprintf('\t%s is currently in %s mode.\n',obj.ID,obj.q),...
                    sprintf('\tOnly Free vehicles can create a new platoon.\n')
                    ]);
                p = [];
                return
            end
            % Create new platon and switch to 'Leader' mode
            if nargin<2
                p = platoon(obj);
            else
                 p = platoon(obj,nmax);
            end
            obj.q = 'Leader';
            obj.platoon = p;
            obj.idx = 1;
        end
        
        function joinPlatoon(obj,platoon)
            % oin together from two different highways/lanes/routes
            
            % Check if vehicle is Leader or Free
            
            % To avoid use of threads, we do not simulate message passing
            % here and instead assume that the following operations are
            % acknowledged and agreed upon by all vehicles/platoons involved.
            if (strcmp(obj.q,'Free'))
                if platoon.n < platoon.nmax
                    % Update vehicle
                    obj.q        = 'Follower';
                    obj.platoon  = platoon;
                    obj.idx      = platoon.n + 1;
                    % Update platoon structure
                    platoon.n                    = platoon.n + 1;
                    platoon.IDvehicle{platoon.n} = obj.ID;
                    platoon.vehicle{platoon.n}   = obj;
                else
                    warning([
                    sprintf('\nPlatoon join failure.\n'),...
                    sprintf('\t %s could not join %s.\n',obj.ID,platoon.ID),...
                    sprintf('\t #vehicles in %s: %d\n', platoon.ID,platoon.n),...
                    sprintf('\t max #vehicles in %s: %d\n',platoon.ID,platoon.nmax)
                    ]);
                end
            elseif (strcmp(obj.q,'Leader'))
                if obj.platoon == platoon % Do not allow self-joining
                    warning(sprintf('%s is already leading %s.\n',obj.ID,platoon.ID))
                    return
                end
                if platoon.n + obj.platoon.n <= platoon.nmax
                    % Change vehicle mode
                    obj.q        = 'Follower';
                    % Update platoon structure and all trailing vehicles
                    platoon.annex(obj.platoon)
                else
                    warning([
                    sprintf('\nPlatoon merge failure.\n'),...
                    sprintf('\t %s could not join %s.\n',obj.platoon.ID,platoon.ID),...
                    sprintf('\t #vehicles in %s: %d\n', platoon.ID,platoon.n),...
                    sprintf('\t #vehicles in %s: %d\n', obj.platoon.ID,obj.platoon.n),...
                    sprintf('\t max #vehicles in %s: %d\n',platoon.ID,platoon.nmax)
                    ]);
                end
            else
                warning([
                    sprintf('Cannot join new platoon.\n'),...
                    sprintf('\t%s is currently in %s mode.\n',obj.ID,obj.q),...
                    sprintf('\tOnly Free or Leader vehicles can join a platoon.\n')
                    ]);
            end
            
        end
        
        function p = splitPlatoon(obj)
            % Split current platoon in two and become Leader of the trailing platoon
            if ~strcmp(obj.q,'Follower')
                warning([
                    sprintf('Cannot split from platoon.\n'),...
                    sprintf('\t%s is currently in %s mode.\n',obj.ID,obj.q),...
                    sprintf('\tOnly Follower vehicles can split form a platoon.\n')
                    ]);
                p = [];
                return
            end
            % Create new platoon (max size to allow re-joining)
            p = platoon(obj, obj.platoon.nmax - (obj.idx-1) );
            
            % Split all lists and update info
            p.vehicle             = obj.platoon.vehicle(obj.idx:end);
            obj.platoon.vehicle   = obj.platoon.vehicle(1:obj.idx-1);
            p.IDvehicle           = obj.platoon.IDvehicle(obj.idx:end);
            obj.platoon.IDvehicle = obj.platoon.IDvehicle(1:obj.idx-1);
            p.n = obj.platoon.n - (obj.idx - 1);
            obj.platoon.n = obj.idx - 1;
            
            % Update follower vehicles in new trailing platoon
            % (after this point the function loses original platoon handle)
            obj.q = 'Leader';
            for i = 1:p.n
                p.vehicle{i}.platoon = p;
                p.vehicle{i}.idx = i;
            end
        end
        
        function abandonPlatoon(obj)
            % Break away from current platoon without splitting it

            % Update index for trailing vehicles
            for i = obj.idx+1:obj.platoon.n
                    obj.platoon.vehicle{i}.idx = i-1;
            end

            % Update platoon lists
            obj.platoon.vehicle = [obj.platoon.vehicle(1:obj.idx-1), obj.platoon.vehicle(obj.idx+1:end)];
            obj.platoon.IDvehicle = [obj.platoon.IDvehicle(1:obj.idx-1), obj.platoon.IDvehicle(obj.idx+1:end)];
            obj.platoon.n = obj.platoon.n - 1;

            % If leader abandons platoon, designate first follower as new leader
            if obj.idx == 1 
                obj.platoon.vehicle{1}.q = 'Leader';
            end
            
            % Update vehicle mode
            obj.q = 'Free';
            obj.platoon = [];
            obj.platoon.idx = 0;
        end
            
        
%         function u = joinslashmerge()
%             % close the following distance with vehicles in the same route
%         end
        

%         function u = split(highway)
%             % Split from group and move onto highway
%         end
        
    end
end