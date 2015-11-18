function u = joinPlatoon(obj, platoon)
% u = joinPlatoon(obj, platoon)
% method of TFM class

% Get first available slot
idx = platoon.getFirstEmptySlot;

% Compute phantom position
pPh = platoon.phantomPosition(obj, tfm.ipsd, idx);

% Compute control
u = getToRelpos(obj, veh, platoon.vehicles{1}, relpos, debug);

% After merging, update properties
if isempty(u)
  
end
end