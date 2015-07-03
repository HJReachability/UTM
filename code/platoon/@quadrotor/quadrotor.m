classdef quadrotor < handle
% Note: Since quadrotor is a "handle class", we can pass on
% handles/pointers to other quadrotor objects
% e.g. a.platoon.leader = b (passes b by reference, does not create a copy)
% Also see constructor

    properties
        ID          % ID number (global, unique)
        
        tauInt      % Separation distance with vehicles inside platoon (depends on mode)
        tauExt      % Separation distance with objects outside platoon
        
        x           % State (4D)
        u           % Recent control signal
        
        xhist       % History of state
        uhist       % History of control
                
        q           % Mode
                    %   1: Free
                    %   2: Follower
                    %   3: Leader
                    %   4: Faulty
                    
        nx          % Dimension of state
        nu          % Dimension of control
        
        dt          % Sampling time
                
        uMin        % Control bounds
        uMax
        
        vMax        % speed bounds
        vMin   
        
        A           % Dynamics
        B
                            
        msg         % Cell array of structures for V2V/V2I messages
        idx         % Vehicle index in platoon (determines phantom position)
        idxJoin     % Vehicle index in platoon it's trying to join 
        
        p                  % Pointer to platoon
        pJoin              % platoon that vehicle is trying to join
        Leader             % Pointer to leader of this quadrotor
        FQ                 % Pointer to quadrotor in front (self if leader)
        BQ                 % Pointer to quadrotor behind (self if tail)

        %         safeV       % Footprint from reachable set
        mergeHighwayV      % Value function for merging onto highway
        mergePlatoonV      % Value function for merging onto platoon
        
        % Figure handles
        hpxpy           % Position
        hpxpyhist       % Position history
        hvxvy           % Velocity
        hvxvyhist       % Velocity history
		hsafeV          % Safety set
        hmergeHighwayV  % Merging reachable set
        hmergePlatoonV  % Merging reachable set
        
        
        % Safety indicators
        safeI           % w.r.t. Intruder
        safeIhist       % w.r.t. Intruder history
        safeFQ          % w.r.t. FQ
        safeFQhist      % w.r.t. FQ history
        safeBQ          % w.r.t. BQ
        safeBQhist      % w.r.t. BQ history
        
    end
    
    
    properties(Constant) 
                
        pdim = [1 3] % Index of position state variables
        vdim = [2 4] % Index of velocity state variables
        
    end
    
    methods
        function obj = quadrotor(ID, dt, x, reachInfo)
            % obj = quadrotor(ID, dt, x, reachInfo)
            %
            % Constructor. Creates a quadrotor object with a unique ID,
            % sampling time dt, state x, and reachable set information
            % reachInfo
            %
            % Dynamics:
            %    \dot{p}_x = v_x
            %    \dot{v}_x = u_x
            %    \dot{p}_y = v_y
            %    \dot{v}_y = u_y
            %       uMin <= u_x <= uMax
            %
            % Inputs:   ID        - unique ID of the quadrotor
            %           dt        - sampling time
            %           x         - state: [xpos; xvel; ypos; yvel]
            %           reachInfo - reachable set information
            %                    .uMax, .uMin - max and min inputs
            %                    .vMax, .vMin - max and min velocities
            %
            % Output:   obj       - a quadrotor object
            %
            % Mo Chen, Qie Hu, 2015-05-22
            
            % Preliminary
            obj.ID = ID;
            obj.dt = dt;
            obj.q = 'Free';
            obj.tauInt = 1.5;
            obj.tauExt = 3;
            
            % Control bounds and Reachable set
            if nargin<4
                obj.uMax = 1.7;
                obj.uMin = -1.7;  
                obj.vMax = 5;
                obj.vMin = -5;
%                 obj.safeV = [];
            else
                obj.uMax = reachInfo.uMax;
                obj.uMin = reachInfo.uMin;
                obj.vMax = reachInfo.vMax;
                obj.vMin = reachInfo.vMin;
%                 obj.safeV = reachInfo.safeV;
            end
            
            
            % A matrix
            obj.A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
            
            % B matrix
            obj.B = [0 0; 1 0; 0 0; 0 1];
                        
            % Dimensions
            [obj.nx, obj.nu] = size(obj.B);
            
            % Initial state and state history
            if nargin < 3
                obj.x = zeros(obj.nx, 1);
            else
                obj.x = x;
            end
            obj.xhist = obj.x;
            
            % Initial control
            obj.u = zeros(obj.nu, 0);
            obj.uhist = obj.u;
            
            % Handle to platoon and index if appropriate
            obj.p = [];
            obj.Leader = [];
            obj.FQ = [];
            obj.BQ = [];
            obj.idx = 0;
            
            % Merging onto highway
            obj.mergeHighwayV = [];
            obj.mergePlatoonV = [];
            
            % Figure handles
            obj.hpxpy = [];                % Position
            obj.hpxpyhist = [];            % Position history
            obj.hvxvy = [];                % Velocity
            obj.hvxvyhist = [];            % Velocity history
			obj.hsafeV = {[],[],[],[],[]}; % Safe reachable sets, up to 5 (w.r.t to each vehicle in platoon)
            obj.hmergeHighwayV = [];       % merge highway reachable set
            obj.hmergePlatoonV = [];       % merge platoon reachable set
            
            % Safety indicators (assume initially safe)
            obj.safeI = true;
            obj.safeFQ = true;
            obj.safeBQ = true;
            obj.safeIhist = obj.safeI;
            obj.safeFQhist = obj.safeFQ;
            obj.safeBQhist = obj.safeBQ;
        end        
    end
end