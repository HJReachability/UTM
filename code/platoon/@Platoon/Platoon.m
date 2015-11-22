classdef Platoon < Node
  
  properties
    % ID of platoon (should be equal to ID of leader vehicle)
    ID
    
    % Cell array of handles to vehicles currently in the platoon
    vehicles
    
    % Vehicle spacing
    veh_spacing
    
    % Maximum allowable number of vehicles in platoon
    nmax
    
    % List of slots in platoon (vector of length nmax)
    %   1:  Slot is occupied
    %   0:  Slot is empty
    %   -1: Slot is pending (a vehicle is
    %       attempting to occupy the slot by joining platoon
    slotStatus
    
    % handle to the highway the platoon is on
    hw
    
    % Number of vehicles in platoon
    n
    
    % Front and back platoons (points to self if there are no from and back
    % platoons)
    FP
    BP
    
  end
  
  methods
    function obj = Platoon(leader, hw, tfm, nmax)
      % function obj = Platoon(leader, hw, nmax, followTime)
      % Constructor for Platoon object
      %
      % Must specify a leader and a highway!
      %
      % Inputs:  leader     - leader vehicle (the first vehicle in the
      %                       platoon)
      %          hw         - highway of platoon
      %          tfm        - traffic flow manager
      %          nmax       - maximum number of vehicles
      %
      % Output: obj         - platoon object
      %
      % Mo Chen, 2015-07-06
      % Modified: Qie Hu, 2015-07-17
      % Modified: Mo Chen, 2015-11-18

      %% Make sure leader is within the width of the highway
      [~, dist] = hw.highwayPos(leader.getPosition);
      if dist > hw.width
        error('Vehicle is too far to be added to platoon on this highway!')
      end
      
      %% Maximum number of vehicles
      if nargin<4
        nmax = 5;
      end
      
      obj.nmax = nmax;
      
      %% Preliminary
      obj.ID = leader.ID;
      obj.n = 1;
      
      % Intra-vehicle spacing
      obj.veh_spacing = tfm.ipsd;
      
      %% Vehicle list and slot status
      obj.vehicles = cell(obj.nmax, 1);
      obj.vehicles{1} = leader;
      obj.slotStatus = zeros(nmax, 1);
      obj.slotStatus(1) = 1;
      
      % Set vehicle mode to leader
      obj.vehicles{1}.q = 'Leader';
      obj.vehicles{1}.idx = 1;
      obj.vehicles{1}.p = obj;
      
      % Vehicle in front and behind
      leader.FQ = leader;
      leader.BQ = leader;
      
      %% Add platoon to highway
      hw.addPlatoon(obj);
      
      % Initialize platoon pointers to self
      obj.FP = obj;
      obj.BP = obj;
      
     end
  end
end