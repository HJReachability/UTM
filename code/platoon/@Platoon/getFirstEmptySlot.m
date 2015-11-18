function idx = getFirstEmptySlot(obj)
% idx = getFirstEmptySlot(obj)
% method of Platoon class
%
% Returns the first empty slot, if available; otherwise, returns -1

if length(obj.vehicles) >= nmax
  idx = -1;
  return;
end

idx = find(~obj.slotStatus, 1, 'first');

end