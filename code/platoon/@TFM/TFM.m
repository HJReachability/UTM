classdef TFM < handle
  % Traffic flow manager
  properties
    % Highways
    hws = {};
    hw_speed = 3;
    
    % active agents
    aas = {};
    
    % safety time
    safetyTime = 2;
    
    % Thresholds for being considered in target set or in reachable set
    ttt = 1; % target threshold time
    rtt = 8;    % reachable set threshold time
    
    % Frequency of state updates to the system
    dt = 0.1;
    
    %% Quadrotor reachable sets
    % Quadrotor create platoon reachable set
    qr_create_platoon_V
    
    % Quadrotor join platoon reachable set
    qr_join_platoon_V
    
    % Quadrotor-quadrotor safety reachable set
    qr_qr_safeV
    
    %% Plane reachable sets
    % Plane create platoon reachable set
    pl_create_platoon_V
    
    % Plane join platoon reachable set
    pl_join_platoon_V
    
    % Plane-plane safety reachable set
    pl_pl_safe_V
    
  end
  
  % No explicit constructor
end % end classdef