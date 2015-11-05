classdef Platoon < handle
  
  properties
    % ID of platoon (should be equal to ID of leader vehicle)
    ID
    
    % Cell array of handles to vehicles currently in the platoon
    vehicles
    
    % Maximum allowable number of vehicles in platoon
    nmax
    
    % Cell array of handles to vehicles attempting to join the platoon
    vJoin
    
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
    
    % Last occupied slot index
    loIdx
    
    % Separation time for followers
    followTime
    
    % Front and back platoons (points to self if there are no from and back
    % platoons)
    FP
    BP
    
    % Footprint of platoon
    safeV
    
    %Liveness reachable set for tracking platoon positions
    liveV
  end
  
  methods
    function obj = platoon(leader, hw, nmax, followTime,FourD)
      % function obj = platoon(leader, hw, nmax, followTime)
      % Constructor for platoon object
      %
      % Must specify a leader and a highway!
      %
      % Inputs:  leader     - leader vehicle (the first vehicle in the
      %                       platoon)
      %          hw         - highway of platoon
      %          nmax       - maximum number of vehicles
      %          followTime - required following time for other platoons
      %                       behind this one
      %
      % Output: obj         - platoon object
      %
      % Mo Chen, 2015-07-06
      % Modified: Qie Hu, 2015-07-17
      
      % Maximum number of vehicles
      if nargin<3
        nmax = 5;
      end
      obj.nmax = nmax;
      
      if nargin<4
        followTime = 2;
      end
      obj.followTime = followTime;
      
      % Preliminary
      obj.ID = leader.ID;
      obj.n = 1;
      
      % Vehicle list and slot status
      obj.vehicles = cell(obj.nmax, 1);
      obj.vehicles{1} = leader;
      obj.slotStatus = zeros(nmax, 1);
      obj.slotStatus(1) = 1;
      obj.loIdx = 1;
      
      % Point to highway
      obj.hw = hw;
      
      % make highway object point to this platoon
      obj.hw.ps = [obj.hw.ps obj];
      
      % Make sure leader is within the width of the highway
      [~, dist] = obj.hw.highwayPos( ...
        obj.vehicles{1}.x(obj.vehicles{1}.pdim) );
      if dist > obj.hw.width
        error('Vehicle is too far to be added to platoon on this highway!')
      end
      
      % Set vehicle mode to leader
      obj.vehicles{1}.q = 'Leader';
      obj.vehicles{1}.idx = 1;
      obj.vehicles{1}.p = obj;
      obj.vehicles{1}.Leader = obj.vehicles{1};
      
      % Set BQ and FQ to itself for leader
      obj.vehicles{1}.FQ = obj.vehicles{1};
      obj.vehicles{1}.BQ = obj.vehicles{1};
      
      % Initialize platoon pointers to self
      obj.FP = obj;
      obj.BP = obj;
      
      % Remove merge highway value function
      obj.vehicles{1}.mergeHighwayV = [];
      
      if nargin < 5
        FourD=1;
      end
      
      if FourD
        filename = '../../data/quad_liveness_4D.mat';
      else
        filename = '../../data/quad_liveness_2x2D.mat';
      end
      
      load(filename)
      
      obj.liveV=liveV;
      
      obj.vJoin = cell(nmax, 1);
      
    end
  end
end

% function annex(obj,platoon) % Append trailing platoon at the back of obj
% % UNUSED?? Should put this in a separate file
%
% if platoon.FP ~= obj
%   warning([
%     sprintf('Cannot append platoon.\n'),...
%     sprintf('\tPlatoon %s is behind platoon %s.\n',platoon.ID, platoon.FQ.ID),...
%     sprintf('\tCan only append platoons directly behind current platoon.\n')
%     ]);
% end
%
% % Update platoon pointers
% if platoon.BP == platoon
%   obj.BP          = obj;
% else
%   obj.BP          = platoon.BP;
%   platoon.BP.FP   = obj;
% end
%
% % Update vehicle pointers
% obj.vehicle{obj.n}.BQ   = platoon.vehicle{1};
% platoon.vehicle{1}.FQ   = obj.vehicle{obj.n};
%
% % Update follower vehicles in old trailing platoon
% for i=1:platoon.n
%   platoon.vehicle{i}.platoon = obj;
%   platoon.vehicle{i}.idx = obj.n + platoon.vehicle{i}.idx;
% end
% % Concatenate all lists and update info
% obj.n         =  obj.n     +    platoon.n;
% obj.vehicle   = [obj.vehicle    platoon.vehicle];
% obj.IDvehicle = [obj.IDvehicle  platoon.IDvehicle];
% % Delete old platoon object
% delete(platoon)
% end