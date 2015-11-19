function u = joinPlatoon(obj, veh, platoon, tfm)
% u = joinPlatoon(obj, platoon)
% method of TFM class

if ~strcmp(veh.q, 'Free') && ~strcmp(veh.q, 'Leader')
  error('Vehicle must be ''Free'' or ''Leader''!')
end

veh.tfm_status = 'busy';

% Get first available slot
idx = platoon.getFirstEmptySlot;

% Compute phantom position
pPh_rel = platoon.phantomPosition(obj.ipsd, idx);

% Compute control
u = getToRelpos(obj, veh, platoon.vehicles{1}, pPh_rel);

% After merging, update properties
if isempty(u)
  veh.tfm_status = 'idle';
  
  % If vehicle is free, we're done
  if strcmp(veh.q, 'Free')
    platoon.insertVehicle(veh, idx);
    u = veh.followPlatoon(tfm); % Needs to be after vehicle is a follower
    return
  end
  
  % If vehicle is a leader, insert all followers
  % Keep in mind that there may be empty slots
  for i = 1:length(veh.p.vehicles)
    if ~isempty(veh.p.vehicles{i})
      platoon.insertVehicle(veh.p.vehicles{i}, idx);
      idx = idx + 1;
    end
  end
  u = veh.followPlatoon(tfm); % Needs to be after vehicle is a follower
end
end