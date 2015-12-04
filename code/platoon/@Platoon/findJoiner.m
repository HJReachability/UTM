function idx = findJoiner(obj, veh)
% idx = findJoiner(obj, veh)
% method of Platoon class
%
% Returns the index of a joiner if found; otherwise returns empty

idx = [];
for i = 1:length(obj.joiners)
  if veh == obj.joiners{i}
    idx = i;
    return
  end
end

end