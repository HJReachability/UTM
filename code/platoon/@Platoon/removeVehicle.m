function removeVehicle(obj, idx)
% function u = removeVehicle(obj, vehicle)
%
% Remove vehicle from platoon
%
% Inputs: obj     - platoon object
%         vehicle - vehicle to be removed
%
% Qie Hu, 2015-07-01
% Modified: Mo Chen, 2015-11-19

%% Removing the last vehicle of the platoon
if obj.n == 1
  % Update vehicle properties
  obj.vehicles{idx}.q = 'Free';
  obj.vehicles{idx}.FQ = [];
  obj.vehicles{idx}.BQ = [];
  obj.vehicles{idx}.idx = [];

  % Remove pointers to this platoon and delete
  obj.vehicles{idx}.p = [];
  obj.hw.removePlatoon(obj.idx);
  delete obj;
  return
end

%% Removing a follower
if strcmp(obj.vehicles{idx}.q, 'Follower')
  % Update vehicle properties
  obj.vehicles{idx}.q = 'Free';
  obj.vehicles{idx}.FQ = [];
  obj.vehicles{idx}.BQ = [];
  obj.vehicles{idx}.idx = [];
  
  % If there are no vehicles behind, then make the front vehicle point to
  % itself. Otherwise, there is a vehicle behind this one. In this case, 
  % point vehicle in front to vehicle behind, and vice versa
  if isempty(obj.vehicles{idx}.BQ)
    obj.vehicles{idx}.FQ.BQ = obj.vehicles{idx}.FQ;
  else
    obj.vehicles{idx}.FQ.BQ = obj.vehicles{idx}.BQ;
    obj.vehicles{idx}.BQ.FQ = obj.vehicles{idx}.FQ;
  end
  
  % Update platoon properties
  obj.vehicles{idx} = [];
  obj.slotStatus(idx) = 0;
  obj.n = obj.n - 1;
end

%% Removing a leader
if strcmp(obj.vehicles{idx}.q, 'Leader')
  % Update vehicle properties
  obj.vehicles{idx}.q = 'Free';
  obj.vehicles{idx}.FQ = [];
  obj.vehicles{idx}.BQ = [];
  obj.vehicles{idx}.idx = [];
  
  % Promote new leader
  obj.vehicles{idx}.BQ.q = 'Leader';
  obj.vehicles{idx}.BQ.FQ = obj.vehicles{idx}.BQ;
  
  % Update platoon properties
  obj.ID = obj.vehicles{idx}.BQ.ID;
  obj.n = obj.n - 1;
  
  % Shift list of vehicles and update their indices
  n_shift = obj.vehicles{idx}.BQ.idx - 1;
  for i = 1:length(obj.vehicles)-n_shift
    obj.vehicles{i} = obj.vehicles{i + n_shift};
    obj.vehicles{i}.idx = i;
  end
end

end