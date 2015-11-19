function idx = getFirstEmptySlot(obj)
% idx = getFirstEmptySlot(obj)
% method of Platoon class
%
% Returns the first empty slot, if available; otherwise, returns empty

if all(obj.slotStatus)
  idx = [];
  return;
end

idx = find(~obj.slotStatus, 1, 'first');

end