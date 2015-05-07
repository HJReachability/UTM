% function u = followPlatoon(obj, safeV, highwayPath)
function u = followPlatoon(obj, highwayPath)

%% update platoon information

% if obj.flag_newLeader
%     obj.Leader = obj.msg.Leader;
%     obj.platoon_index = obj.msg.platoon_index;
%     obj.BQ.flag_newLeader = 1;
%     obj.BQ.msg.platoon_index = obj.platoon_index + 1;
%     obj.flag_newLeader = 0;
%     obj.msg = [];
% end
% 
% if obj.flag_newFQ
%     obj.FQ = obj.msg.FQ;
%     obj.platoon_index = obj.msg.platoon_index;
%     obj.BQ.flag_newIndex = 1;
%     obj.BQ.msg.platoon_index = obj.platoon_index + 1;
%     obj.flag_newFQ = 0;
%     obj.msg = [];
% end
% 
% if obj.flag_newBQ
%     obj.BQ = obj.msg.BQ;
%     obj.flag_newBQ = 0;
%     obj.msg = [];
% end
% 
% if obj.flag_newIndex
%     obj.platoon_index = obj.msg.platoon_index;
%     obj.BQ.flag_newIndex = 1;
%     obj.BQ.msg.platoon_index = obj.platoon_index + 1;
%     obj.flag_newIndex = 0;
%     obj.msg = [];
% end

%% check safety and define control and platoon actions

% if strcmp(obj.q, 'Leader') | strcmp(obj.q, 'EmergLeader')
%     [safeF, uSafeF] = obj.isSafe(otherPlatoon, obj.tauExt);
% else

% This method is only called on Followers, so obj.FQ ~= obj
% [safeF, uSafeF] = obj.isSafe(obj.FQ, safeV);
% 
% if obj.idx == obj.platoon.n
%     safeB = true;
% else
%     [safeB, uSafeB] = obj.isSafe(obj.BQ, safeV);
% end


% if safeF && safeB
% threshold < followTime to avoid platoon disintegrates on disturbance
% alignment of the platoon based on virtual platoon's heading, set by leader
% (allows leader to command sideways evasions without "turning")
k_p = 10;
k_v = 1;

u = obj.Leader.u ...
    + k_p*(obj.platoon.phantomPosition(highwayPath, obj.idx) - obj.x(obj.pdim))...
    + k_v*(obj.Leader.x(obj.vdim) - obj.x(obj.vdim));

% Acceleration limit
u = max(u, obj.uMin);
u = min(u, obj.uMax);
    
% Speed limit
if obj.x(obj.vdim(1)) > obj.vMax || obj.x(obj.vdim(1)) < obj.vMin
    u(1) = 0;
end

if obj.x(obj.vdim(2)) > obj.vMax || obj.x(obj.vdim(2)) < obj.vMin
    u(2) = 0;
end

% elseif ~safeF & safeB
    % accelerate away from FQ and take over rest of platoon
%     u = uSafeF;
%     obj.splitPlatoon();
    
% elseif safeF & ~safeB
    % accelerate away from BQ and alert leader by flagging platoon
    % (in practice this is done through a message to the leader)
%     u = uSafeB;
%     Leader.flag_safety_request(obj.platoon_index) = 1; %obj.Idx or obj.platoon.ID?
%     Leader.msg.u_safe(obj.platoon_index) = u;
    
% else
    % accelerate up and break out of the platoon (rest of platoon may not need to break apart)
% 	u = obj.emergency_shoot_up_control; % consider pros and cons of DROPPING instead
%     u = obj.abandonPlatoon();
% end

end % end function

