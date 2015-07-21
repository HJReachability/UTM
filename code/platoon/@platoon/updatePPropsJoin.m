function updatePPropsJoin(obj, platoon)
% function updatePPropsJoin(obj, platoon)
%
% Method of platoon class. 
% Platoon successfully joins obj
% Note: platoon may join in the middle of obj
% Updates both platoon and their vehicle properties
%
% Inputs: obj     - platoon object
%         platoon - platoon that is added
%
% Mo Chen, Qie Hu, 2015-07-01
% Modified: Qie Hu, 2015-07-22

% Update number of vehicles in platoon
obj.n = obj.n + platoon.n;

% Update last occupied slot index
obj.loIdx = platoon.vehicles{platoon.loIdx}.idxJoin;

% Occupied slots in platoon
occup_slot = find(platoon.slotStatus == 1)';

for i = occup_slot
    % Confirm vehicle position in platoon
    platoon.vehicles{i}.idx = platoon.vehicles{i}.idxJoin;
    platoon.vehicles{i}.idxJoin = [];
    
    % Update vehicle pointers and join list pointers
    obj.vehicles{platoon.vehicles{i}.idx} = platoon.vehicles{i};
    obj.vJoin{platoon.vehicles{i}.idx} = [];
    
    % Update vehicle list in platoon
    obj.slotStatus(platoon.vehicles{i}.idx) = 1;
    
    % Empty platoon join pointer
    platoon.vehicles{i}.pJoin = [];
end

% Update platoon pointers
if platoon.BP == platoon
    % Platoon has no other platoons behind
    obj.BP          = obj;
    
else
    % Platoon has another platoon behind it
    obj.BP          = platoon.BP;
    platoon.BP.FP   = obj;
    
    if platoon.BP.vehicles{1}.pJoin == platoon
        % Platoon behind it is attempting to join this platoon
        if obj.loIdx + platoon.BP.loIdx <= obj.nmax
            % If there are available slots in obj,
            % update platoon.BP to join obj (at the end)
            for i = find(platoon.BP.slotStatus == 1)'
                platoon.BP.vehicles{i}.pJoin = obj;
                platoon.BP.vehicles{i}.idxJoin = obj.loIdx+i; % Join at the end
            end
        else
            % update platoon.BP to not join platoon
            for i = find(platoon.BP.slotStatus == 1)'
                platoon.BP.vehicles{i}.pJoin = [];
                platoon.BP.vehicles{i}.idxJoin = [];
            end
        end
    end
end

% Update pointers to front and behind vehicles for leader of this platoon
% and the trailing vehicle in front
obj.vehicles{platoon.vehicles{1}.idx-1}.BQ  = platoon.vehicles{1}; % Point previous trailer
platoon.vehicles{1}.FQ                      = obj.vehicles{platoon.vehicles{1}.idx-1};


% Update other vehicle fields in platoon
for i = occup_slot
    obj.vehicles{platoon.vehicles{i}.idx}.q = 'Follower';            % Mode
    obj.vehicles{platoon.vehicles{i}.idx}.p = obj;                   % Platoon pointer
    obj.vehicles{platoon.vehicles{i}.idx}.Leader = obj.vehicles{1};  % Leader pointer
    obj.vehicles{platoon.vehicles{i}.idx}.mergePlatoonV = [];        % merge platoon value function
end

% Delete old platoon object
delete(platoon)



end