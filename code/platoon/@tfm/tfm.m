classdef tfm < handle
  properties
    % Highways
    hws = {};
    
    % Frequency of state updates
    dt = 1;
    
    %% Quadrotor reachable sets
    % Quadrotor create platoon reachable set
    qr_create_platoon_V
    
    % Quadrotor join platoon reachable set
    qr_join_platoon_V
    
    % Quadrotor-quadrotor safety reachable set
    qr_qr_SafeV
    
    %% Plane reachable sets
    % Plane create platoon reachable set
    pl_create_platoon_V
    
    % Plane join platoon reachable set
    pl_join_platoon_V
    
    % Plane-plane safety reachable set
    pl_pl_safe_V
    
  end
  
  methods
    function obj = tfm
    end
    
    function addHighway(obj, hw)
    end

  end % end methods
end % end classdef