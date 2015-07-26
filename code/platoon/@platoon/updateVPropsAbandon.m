function updateVPropsAbandon(obj, vehicle)
% function updateVPropsAbandon(obj, vehicle)
%
% Method of platoon class. Updates vehicle and platoon properties after 
% removing a vehicle from the platoon
%
% Inputs: obj     - platoon object
%         vehicle - vehicle to be removed
%
% Qie Hu, 2015-07-02
% Modified: Qie Hu, 2015-07-25

% Occupied slots in platoon
occup_slot = find(obj.slotStatus == 1)';

if obj.n == 1
    % Vehicle is the only vehicle in its platoon, so when it abandons, its
    % platoon disappears. Update pointers to platoon in front and behind
    if ~isempty(obj.FP)
        obj.FP.BP = obj.BP;
    end
    
    if ~isempty(obj.BP)
        obj.BP.FP = obj.FP; 
    end
    
    delete(obj) 

else
    
     % Platoon has more than one vehicle
    if vehicle.idx == 1 % Leader abandons platoon
        % Designate first follower as new leader
        nextIdx = occup_slot(2);
        obj.vehicles{nextIdx}.q = 'Leader'; 
        obj.ID = obj.vehicles{nextIdx}.ID;
        vehicle.BQ.FQ = vehicle.BQ;
        
    elseif vehicle.idx == obj.loIdx  % Trailing vehicle abandons platoon
        vehicle.FQ.BQ = vehicle.FQ;
        
    else   % A vehicle in the middle abandons platoon
        vehicle.FQ.BQ = vehicle.BQ;
        vehicle.BQ.FQ = vehicle.FQ;
    end
    
    % Update slotStatus for vehicle
    obj.slotStatus(vehicle.idx) = 0;
    
    for i = vehicle.idx+1:obj.loIdx
        if obj.slotStatus(i) == 1
            % Update index for trailing vehicles
            obj.vehicles{i}.idx = i-1;
        end
        
        % Update vehicle positions 
        obj.vehicles{i-1} = obj.vehicles{i};
        
        % join list pointers
        if ~isempty(obj.vJoin{i})
            obj.vJoin{i}.idxJoin = obj.vJoin{i}.idxJoin - 1;
        end
        obj.vJoin{i-1} = obj.vJoin{i};
    end
    
    % Update number of vehicles
    obj.n = obj.n - 1;
    
    % Update last occupied vehicle position
    obj.vehicles{obj.loIdx} = [];
    obj.vJoin{obj.loIdx} = [];
    obj.loIdx = obj.loIdx-1;
    
    % Update vehicle list in obj
    obj.slotStatus(vehicle.idx:end-1) = obj.slotStatus(vehicle.idx+1:end);
    obj.slotStatus(end) = 0;
    
end

% Update vehicle information
vehicle.q = 'Free';
vehicle.p = [];
vehicle.FQ = [];
vehicle.BQ = [];
vehicle.Leader = [];
vehicle.idx = 0;
vehicle.idxJoin = [];

% If vehicle was attempting to join another platoon, free up the slot
if ~isempty(vehicle.pJoin)
    vehicle.pJoin.slotStatus(vehicle.idxJoin) = 0;
    vehicle.pJoin.vJoin{vehicle.idxJoin} = [];
    vehicle.pJoin = [];
end

end