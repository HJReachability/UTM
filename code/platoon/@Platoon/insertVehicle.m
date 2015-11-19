function insertVehicle(obj, veh, idx)
% insertVehicle(obj, veh, idx)
% method of Platoon class
%
% Inserts a vehicle to the platoon object at position idx

if ~isempty(obj.vehicles{idx})
  error(['Position ' num2str(idx) ' already occupied!'])
end

veh.q = 'Follower';
veh.p = obj;
obj.vehicles{idx} = veh;
obj.slotStatus(idx) = 1;
veh.idx = idx;

% Scan lower positions to find the platoon member that's in front
for i = idx-1:-1:1
  if ~isempty(obj.vehicles{i})
    veh.FQ = obj.vehicles{i};
    obj.vehicles{i}.BQ = veh;
    break
  end
end

% Scan upper positions to find the platoon member that's behind
for i = idx+1:length(obj.vehicles)
  if ~isempty(obj.vehicles{i})
    veh.BQ = obj.vehicles{i};
    obj.vehicles{i}.FQ = veh;
    break
  end
end

end