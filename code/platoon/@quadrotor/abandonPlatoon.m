function u = abandonPlatoon(obj)
% Break away from current platoon without splitting it
% Qie Hu, 2015-07-01

if obj.p.n > 1 % Obj is in a platoon with more than one vehicle
    
    if obj.idx == 1 % Leader abandons platoon
        obj.p.vehicle(2).q = 'EmergLeader'; % Designate first follower as new leader
        obj.p.ID = obj.p.vehicle(2).ID;
        obj.BQ.FQ = obj.BQ;
        
    elseif obj.idx == obj.p.n  % Trailing quadrotor abandons platoon
        obj.FQ.BQ = obj.FQ;
        
    else   % A quadrotor in the middle abandons platoon
        obj.FQ.BQ = obj.BQ;
        obj.BQ.FQ = obj.FQ;
    end
    
    % Update index for trailing vehicles
    for i = obj.idx+1:obj.p.n
        obj.p.vehicle(i).idx = i-1;
    end
    
    % Update platoon lists
    obj.p.vehicle = [obj.p.vehicle(1:obj.idx-1), obj.p.vehicle(obj.idx+1:end)];
%     obj.p.IDvehicle = [obj.p.IDvehicle(1:obj.idx-1), obj.p.IDvehicle(obj.idx+1:end)];
    obj.p.n = obj.p.n - 1;
    obj.p.vList = [obj.p.List(1:obj.idx-1), obj.p.vehicle(obj.idx+1:end), 0];
    
    % If there are vehicles trying to join this platoon,
    % update their idxJoin
    vs_tf = ismember(obj.p.vList, -1);
    if max(vs_tf)>0
        vs_idx = 1:obj.p.nmax;
        for i = 1:length(vs_idx)
            v_idx = vs_idx(i);
            if v_idx > obj.idx
                obj.p.vJoin(v_idx).idxJoin = obj.p.vJoin(v_idx).idxJoin - 1;
            end
        end
    end
else
    
    % Obj is the single vehicle in its platoon, so when it abandons, its
    % platoon disappears. Update platoon pointers
    obj.p.FP.BP = obj.p.BP;
    obj.p.BP.FP = obj.p.FP; 
    
end

% Update vehicle information
obj.q = 'Free';
obj.p = [];
obj.FQ = [];
obj.BQ = [];
obj.Leader = [];
u = [];

% If vehicle was attempting to join another platoon, free up the slot
if ~isempty(obj.pJoin)
    obj.pJoin.vList(obj.idxJoin) = 0;
    obj.pJoin.vJoin{obj.idxJoin} = [];
end

end