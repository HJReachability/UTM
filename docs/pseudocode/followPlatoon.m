function u = followPlatoon(platoon, reach)


%% update platoon information

if self.flag_newLeader
    self.Leader = self.msg.Leader
    self.platoon_index = self.msg.platoon_index
    self.BQ.flag_newLeader = 1
    self.BQ.msg.platoon_index = self.platoon_index + 1
    self.flag_newLeader = 0
    self.msg = []
end

if self.flag_newFQ
    self.FQ = self.msg.FQ
    self.platoon_index = self.msg.platoon_index
    self.BQ.flag_newIndex = 1
    self.BQ.msg.platoon_index = self.platoon_index + 1
    self.flag_newFQ = 0
    self.msg = []
end

if self.flag_newBQ
    self.BQ = self.msg.BQ
    self.flag_newBQ = 0
    self.msg = []
end

if self.flag_newIndex
    self.platoon_index = self.msg.platoon_index
    self.BQ.flag_newIndex = 1
    self.BQ.msg.platoon_index = self.platoon_index + 1
    self.flag_newIndex = 0
    self.msg = []
end

FQ = self.FQ
BQ = self.BQ
Leader = self.Leader


%% check safety and define control and platoon actions

if (reach.value_function(FQ.state-self.state) > threshold)  & (reach.value_function(BQ.state-self.state) > threshold)
% threshold < followTime to avoid platoon disintegrates on disturbance
% alignment of the platoon based on virtual platoon's heading, set by leader
% (allows leader to command sideways evasions without "turning")
	u = Leader.control...
        + k_p*(Leader.position + rotation(Leader.platoonHeading)*self.offset - self.position)...
        + k_v*(Leader.velocity - self.velocity)

elseif (reach.value_function(FQ.state-self.state) <= threshold) & (reach.value_function(BQ.state-self.state) > threshold)
    % accelerate away from FQ and take over rest of platoon
    u = reach.optimal_control(FQ.state-self.state)
    self.splitPlatoon(platoon)
    
elseif (reach.value_function(FQ.state-self.state) > threshold) & (reach.value_function(BQ.state-self.state) <= threshold)
    % accelerate away from BQ and alert leader by flagging platoon
    % (in practice this is done through a message to the leader)
    u = reach.optimal_control(BQ.state-self.state)
    Leader.flag_safety_request(self.platoon_index) = 1
    Leader.msg.u_safe(self.platoon_index) = u
    
else
    % accelerate up and break out of the platoon (rest of platoon may not need to break apart)
	u = emergency_shoot_up_control % consider pros and cons of DROPPING instead
    self.abandonPlatoon(FQ,BQ,Leader)
end

end % end function

% % ======================================================================
% % What about vehicles in front of rogue vehicle? (need feedforward control too)
% % To ensure quadrotors can respond to intruders, each quadrotor need to check safe reachable set with intruder
% % How about checking with every quadrotor within a bubble around platoon?
% % For turning, estimate target quadrotor position and velocity along path
% 
% if value of safe reachable set with all quadrotors inside bubble > threshold
% % threshold < followTime to avoid platoon disintegrates on disturbance
% 
% 	u = emergency control + position and velocity error feedback
% 
% else
% 	u = optimal safe control
% 
% 	if  u is conflicting control
% 		TQ increases in altitude and disappears
% 		BQ. set mode to leader
% 
% 	elseif 	u is deceleration
% 		set mode to leader
% 		issue emergency control to quadrotors behind it
% 
% 	else % acceleration
% 		issue emergency control to quadrotors in front of it
% 	end
% 	
% 	TQ.updateState
% 
% end
% 	