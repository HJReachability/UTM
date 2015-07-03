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

if obj.n == 1
    
    % Vehicle is the only vehicle in its platoon, so when it abandons, its
    % platoon disappears. Update platoon pointers
    obj.FP.BP = obj.BP;
    obj.BP.FP = obj.FP; 
    delete(obj)

else
    
     % Platoon has more than one vehicle
    
    if vehicle.idx == 1 % Leader abandons platoon
        obj.vehicle(2).q = 'Leader'; % Designate first follower as new leader
        obj.ID = obj.vehicle(2).ID;
        vehicle.BQ.FQ = vehicle.BQ;
        
    elseif vehicle.idx == obj.n  % Trailing vehicle abandons platoon
        vehicle.FQ.BQ = vehicle.FQ;
        
    else   % A vehicle in the middle abandons platoon
        vehicle.FQ.BQ = vehicle.BQ;
        vehicle.BQ.FQ = vehicle.FQ;
    end
    
    % Update index for trailing vehicles
    for i = vehicle.idx+1:obj.n
        obj.vehicle(i).idx = i-1;
    end
    
    % Update platoon lists
    obj.vehicle = [obj.vehicle(1:vehicle.idx-1), obj.vehicle(vehicle.idx+1:end)];
%     obj.IDvehicle = [obj.IDvehicle(1:vehicle.idx-1), obj.IDvehicle(vehicle.idx+1:end)];
    obj.n = obj.n - 1;
    obj.vList = [obj.vList(1:vehicle.idx-1), obj.vList(vehicle.idx+1:end), 0];
    
    % If there are vehicles trying to join this platoon,
    % update their idxJoin
    vs_tf = ismember(obj.vList, -1);
    if max(vs_tf)>0
        vs_idx = find(vs_tf);
        for i = 1:length(vs_idx)
            v_idx = vs_idx(i);
            if v_idx > vehicle.idx
                obj.vJoin(v_idx).idxJoin = obj.vJoin(v_idx).idxJoin - 1;
            end
        end
    end    
    
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
    vehicle.pJoin.vList(vehicle.idxJoin) = 0;
    vehicle.pJoin.vJoin{vehicle.idxJoin} = [];
    vehicle.pJoin = [];
end

end