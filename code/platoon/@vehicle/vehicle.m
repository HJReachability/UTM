classdef vehicle < handle
  % Vehicle class
  %   Subclasses: quadrotor, Dubins vehicle (under construction)
  
  properties
    ID          % ID number (global, unique)
    
    x           % State (4D)
    u           % Recent control signal
    
    xhist       % History of state
    uhist       % History of control
    
    % Mode
    %   'Free'
    %   'Follower'
    %   'Leader'
    %   'Faulty'
    q = 'Free';
    
    p           % Pointer to platoon
    idx         % Vehicle index in platoon (determines phantom position)
    
    pJoin   % platoon that vehicle is trying to join
    idxJoin % Vehicle index in platoon it's trying to join
    
    Leader             % Pointer to leader of this quadrotor
    FQ                 % Pointer to quadrotor in front (self if leader)
    BQ                 % Pointer to quadrotor behind (self if tail)
    
    mergeHighwayV      % Value function for merging onto highway
    mergePlatoonV      % Value function for merging onto platoon
    
    %% Figure handles
    hpxpy           % Position
    hpxpyhist       % Position history
    hvxvy           % Velocity
    hvxvyhist       % Velocity history
    
    % Safety sets (with respect to 5 nearest vehicles, or vehicles in the 
    % same platoon; + intruder/faulty)
    hsafeV = cell(6,1);
        
    hmergeHighwayV  % Merging reachable set
    hmergePlatoonV  % Merging reachable set

    %% Safety indicators
    % Cell structure containing pointers to the vehicles with whome safety 
    % should be checked - need to merge this with the stuff below
    % eventually
    sList = {};
    
    safeI     = true % w.r.t. Intruder
    safeIhist = true % w.r.t. Intruder history
    safeFQ    = true % w.r.t. FQ
    safeFQhist = true % w.r.t. FQ history
    safeBQ     = true % w.r.t. BQ
    safeBQhist = true % w.r.t. BQ history
  end % end properties

  % No constructor in vehicle class. Use constructors in the subclasses
  
end % end class