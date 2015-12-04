function idx = addJoiner(obj, veh)
% idx = addJoiner(obj, veh)
% method of Platoon class

%% Update vehicle's old target platoon if needed
if ~isempty(veh.pJoin)
  prev_idx = veh.pJoin.findJoiner(veh);
  veh.pJoin.removeJoiner(prev_idx);
end
veh.pJoin = obj;

%% Update vehicle's old platoon if needed
if ~isempty(veh.p)
  veh.p.removeVehicle(veh.idx);
end

%% Get first available position in new platoon
idx = obj.getFirstEmptySlot;

if isempty(idx)
  idx = [];
  warning('No available slots for joining!')
  return
end

%% Update joiner list and slot status in new platoon
obj.slotStatus(idx) = -1;
obj.joiners{idx} = veh;
end