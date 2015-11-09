classdef Vehicle < handle
  % Vehicle class
  %   Subclasses: quadrotor, Dubins vehicle (under construction)
  
  properties
    ID          % ID number (global, unique)
    
    x           % State
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
    
    h_abs_target_V     % value function for getting to an absolute target

    mergePlatoonV      % Value function for merging onto platoon
    
    %% Figure handles
    hpxpy           % Position
    hpxpyhist       % Position history
    hvxvy           % Velocity
    hvxvyhist       % Velocity history
    
    % Position velocity (so far only used in DoubleInt)
    hpxvx 
    hpxvxhist
    
    % Safety sets
    hsafeV         % figure handles
    safeV_vehicles % list of corresponding vehicles

    hmergeHighwayV  % Merging reachable set
    hmergePlatoonV  % Merging reachable set

    %% Safety indicators
    safeI     = true % w.r.t. Intruder
    safeIhist = true % w.r.t. Intruder history
    safeFQ    = true % w.r.t. FQ
    safeFQhist = true % w.r.t. FQ history
    safeBQ     = true % w.r.t. BQ
    safeBQhist = true % w.r.t. BQ history
  end % end properties

  % No constructor in vehicle class. Use constructors in the subclasses
  
end % end class