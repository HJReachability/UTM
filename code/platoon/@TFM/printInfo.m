function printInfo(obj)
% printInfo(obj)
% Method of TFM class

disp('TFM Info:')
disp(['  Highway speed = ' num2str(obj.hw_speed)])
disp(['  Intra-platoon separation distance = ' num2str(obj.ipsd)])
disp(['  Number of registered vehicles = ' num2str(length(obj.aas))]);
disp(['  Collision radius = ' num2str(obj.cr)]);
disp(['  Target threshold time = ' num2str(obj.ttt)]);
disp(['  Reachable set threshold time = ' num2str(obj.rtt)]);
disp(['  Period of zero order hold = ' num2str(obj.dt)]);
end