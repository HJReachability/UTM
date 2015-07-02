function updatePlatoonProps(obj, platoon)
% function updateVehicleProps(obj, vehicle)
%
% Method of platoon class. 
% Platoon successfully joins obj
% Updates both platoon and their vehicle properties
%
% Inputs: obj     - platoon object
%         platoon - platoon that is added
%
% Mo Chen, Qie Hu, 2015-07-01

% Update number of vehicles in platoon
obj.n = obj.n + platoon.n;

% Update vehicle pointer list
if platoon.vehicle(1).idxJoin == obj.vehicle(end).idx
    % If the new vehicle's position index already exists, something went
    % terribly wrong...
    error('The vehicle joining the platoon is already in the platoon?')
    
else
    % Otherwise, recreate vehicle list by inserting vehicle
    % after the last vehicle in the current list with position index
    % smaller than the current vehicle to be added
    l_idx = find([obj.vehicle.idx] < platoon.vehicle(1).idxJoin, 1, 'last');
    obj.vehicle = [obj.vehicle(1:l_idx); platoon.vehicle; obj.vehicle(l_idx+1:end)];
end

% Update vehicle list in platoon
c_idx = l_idx + 1;
obj.vList(c_idx : (c_idx+platoon.n-1)) = 1;

% Update platoon pointers
if platoon.BP == platoon
    % Platoon has no other platoons behind
    obj.BP          = obj;
    
else
    % Platoon has another platoon behind it
    obj.BP          = platoon.BP;
    platoon.BP.FP   = obj;
    
    if platoon.BP.vehicle(1).pJoin == platoon
        % Platoon behind it is attempting to join this platoon
        % update to join obj instead at the end
        for i = 1:platoon.BP.n
            platoon.BP.vehicle(i).pJoin = obj;
            platoon.BP.vehicle(i).idx = obj.n+i;
        end
        
    end
end

% Update pointers to front and behind vehicles for leader of this platoon
% and the trailing vehicle in front
obj.vehicle(l_idx).BQ   = platoon.vehicle(1); % Point previous trailer
platoon.vehicle(1).FQ   = obj.vehicle(l_idx);

% If there's a vehicle behind, update front and behind pointers for this
% vehicle and the vehicle behind
if ~isempty(obj.vehicle(c_idx+platoon.n:end))
    obj.vehicle(c_idx+platoon.n).FQ     = platoon.vehicle(end);
    platoon.vehicle(end).BQ             = obj.vehicle(c_idx+platoon.n);
end

% Update all vehicles in old trailing platoon
for i = 1:platoon.n
    obj.vehicle(l_idx+i).q = 'Follower';            % Mode
    obj.vehicle(l_idx+i).p = obj;                   % Platoon pointer
    obj.vehicle(l_idx+i).idx = l_idx + i;           % Vehicle index
    obj.vehicle(l_idx+i).Leader = obj.vehicle(1);   % Leader pointer
    obj.vehicle(l_idx+i).mergePlatoonV = [];        % merge platoon value function
    obj.vehicle(l_idx+i).pJoin = [];                % platoon to join pointer
    obj.vJoin{l_idx+i} = [];                        % pointer to vehicles attempting to join
end

% obj.vehicle(c_idx : (c_idx+platoon.n-1)).q = 'Follower';            % Mode
% obj.vehicle(c_idx : (c_idx+platoon.n-1)).p = obj;                   % Platoon pointer
% obj.vehicle(c_idx : (c_idx+platoon.n-1)).idx = l_idx + i;           % Vehicle index
% obj.vehicle(c_idx : (c_idx+platoon.n-1)).Leader = obj.vehicle(1);   % Leader pointer
% obj.vehicle(c_idx : (c_idx+platoon.n-1)).mergePlatoonV = [];        % merge platoon value function
% obj.vehicle(c_idx : (c_idx+platoon.n-1)).pJoin = [];                % platoon to join pointer

% Delete old platoon object
delete(platoon)



end