function removeVehicle(obj, idx)
% function removeVehicle(obj, idx)
%
% Remove vehicle from platoon
%
% Inputs: obj - platoon object
%         idx - index of vehicle to be removed
%
% Qie Hu, 2015-07-01
% Modified: Mo Chen, 2015-11-19

vehToRemove = obj.vehicles{idx};

%% Removing the last vehicle of the platoon
if obj.n == 1
  % Update vehicle properties
  vehToRemove.q = 'Free';
  vehToRemove.FQ = [];
  vehToRemove.BQ = [];
  vehToRemove.idx = [];

  % Remove pointers to this platoon and delete
  vehToRemove.p = [];
  obj.hw.removePlatoon(obj.idx);
  delete obj;
  return
end

%% Removing a follower
if strcmp(vehToRemove.q, 'Follower')
  % If there are no vehicles behind, then make the front vehicle point to
  % itself. Otherwise, there is a vehicle behind this one. In this case, 
  % point vehicle in front to vehicle behind, and vice versa
  vehToRemove.FQ.BQ = vehToRemove.BQ;
  if ~isempty(vehToRemove.BQ)
    vehToRemove.BQ.FQ = vehToRemove.FQ;
  end

  % Update platoon properties
  obj.vehicles{idx} = [];
  obj.slotStatus(idx) = 0;
  obj.n = obj.n - 1;
  
  % Update vehicle properties
  vehToRemove.q = 'Free';
  vehToRemove.FQ = [];
  vehToRemove.BQ = [];
  vehToRemove.idx = [];
  vehToRemove.p = [];
  return
end

%% Removing a leader
if strcmp(vehToRemove.q, 'Leader')
  % Promote new leader
  vehToRemove.BQ.q = 'Leader';
  vehToRemove.BQ.FQ = vehToRemove.BQ;
  
  % Update platoon properties
  obj.ID = vehToRemove.BQ.ID;
  obj.n = obj.n - 1;
  
  % Shift list of vehicles and update their indices
  n_shift = vehToRemove.BQ.idx - 1;
  for i = 1:length(obj.vehicles)
    if i + n_shift > obj.nmax
      obj.vehicles{i} = [];
      continue
    end
    
    obj.vehicles{i} = obj.vehicles{i + n_shift};
    obj.slotStatus(i) = obj.slotStatus(i + n_shift);
    if ~isempty(obj.vehicles{i + n_shift})
      obj.vehicles{i}.idx = i;
    end
  end
  
  % Update vehicle properties
  vehToRemove.q = 'Free';
  vehToRemove.FQ = [];
  vehToRemove.BQ = [];
  vehToRemove.idx = [];
  vehToRemove.p = [];
  return
end

end