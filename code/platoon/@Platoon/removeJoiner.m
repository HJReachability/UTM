function removeJoiner(obj, idx)
% removeJoiner(obj, idx)
% method of Platoon class
%
% Removes a joiner from the joiners list

% Free up the slot
obj.slotStatus(idx) = 0;
obj.joiners{idx}.pJoin = [];
obj.joiners{idx} = [];

end