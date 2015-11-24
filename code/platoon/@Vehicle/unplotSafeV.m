function unplotSafeV(obj, others)
% function unplotSafeV(obj, others)
%
% Unplot safe region around others that obj must stay out of in order to
% be safe.
%
% Inputs: obj    - this quadrotor
%         others - list of quadrotors for which reachable set plots should
%                  be turned off
%
% Qie Hu, 2015-07-25
% Modified: Mo Chen, 2015-10-20

% Check input
others = checkVehiclesList(others, 'quadrotor');

% Go through vehicles list and compare with list of plotted vehicles. 
% Turn off plots for matching vehicles
for i = 1:length(others)
  for j = 1:length(obj.safeV_vehicles)
    if isequal(others{i}, obj.safeV_vehicles{j})
      obj.hsafeV{j}.Visible = 'off';
    end
  end
end

end
