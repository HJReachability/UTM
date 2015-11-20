function printInfo(obj)

disp('Vehicle Info:')
disp(['  Type = ' class(obj)])
disp(['  ID = ' num2str(obj.ID)])
disp(['  Mode = ' obj.q]);
disp(['  Position = ' num2str(obj.getPosition')])
disp(['  Velocity = ' num2str(obj.getVelocity')])
disp(['  Heading = ' num2str(obj.getHeading)])
disp(['  Last control = ' num2str(obj.u')])
disp(['  tfm status = ' obj.tfm_status])

if ~isempty(obj.p)
  disp(['  Platoon ID = ' num2str(obj.p.ID)])
  disp(['  FQ ID = ' num2str(obj.FQ.ID)])
  disp(['  BQ ID = ' num2str(obj.BQ.ID)])
else
  disp('  Not in a platoon')
end


end