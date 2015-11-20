function printInfo(obj)
% printInfo(obj)
% Method of Platoon class

disp('Platoon Info:')
disp(['  ID = ' num2str(obj.ID)])

veh_str = '  Vehicles:';
for i = 1:length(obj.vehicles)
  if ~isempty(obj.vehicles{i})
    veh_str = [veh_str ' ' num2str(obj.vehicles{i}.ID)];
  end
end
disp(veh_str)

disp(['  Slot status = ' num2str(obj.slotStatus')])
disp(['  n = ' num2str(obj.n)])
disp(['  FP ID = ' num2str(obj.FP.ID)])
disp(['  BP ID = ' num2str(obj.BP.ID)])
end