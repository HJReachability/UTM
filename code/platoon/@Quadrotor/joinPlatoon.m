% function joinPlatoon(obj,platoon)
% % ------ UNUSED ------
% % Join together from two different highways/lanes/routes
% 
% % Check if vehicle is Leader or Free
% 
% % To avoid use of threads, we do not simulate message passing
% % here and instead assume that the following operations are
% % acknowledged and agreed upon by all vehicles/platoons involved.
% if (strcmp(obj.q,'Free'))
%     if platoon.n < platoon.nmax
%         % Update trailing vehicle of platoon
%         platoon.vehicle{platoon.n}.BQ      = obj; %Point tail of platoon to new follower
%         % Update this vehicle
%         obj.q        = 'Follower';
%         obj.platoon  = platoon;
%         obj.Leader   = platoon.vehicle{1};
%         obj.FQ       = platoon.vehicle{platoon.n};
%         obj.BQ       = obj;
%         obj.idx      = platoon.n + 1;
%         % Update platoon structure
%         platoon.n                    = platoon.n + 1;
%         platoon.IDvehicle{platoon.n} = obj.ID;
%         platoon.vehicle{platoon.n}   = obj;
%     else
%         warning([
%             sprintf('\nPlatoon join failure.\n'),...
%             sprintf('\t %s could not join %s.\n',obj.ID,platoon.ID),...
%             sprintf('\t #vehicles in %s: %d\n', platoon.ID,platoon.n),...
%             sprintf('\t max #vehicles in %s: %d\n',platoon.ID,platoon.nmax)
%             ]);
%     end
% elseif (strcmp(obj.q,'Leader')) | (strcmp(obj.q,'EmergLeader'))
%     if obj.platoon == platoon % Do not allow self-joining
%         warning(sprintf('%s is already leading %s.\n',obj.ID,platoon.ID))
%         return
%     end
%     if platoon.n + obj.platoon.n <= platoon.nmax
%         % Change vehicle mode
%         obj.q        = 'Follower';
%         % Update platoon structure and all trailing vehicles
%         platoon.annex(obj.platoon)
%     else
%         warning([
%             sprintf('\nPlatoon merge failure.\n'),...
%             sprintf('\t %s could not join %s.\n',obj.platoon.ID,platoon.ID),...
%             sprintf('\t #vehicles in %s: %d\n', platoon.ID,platoon.n),...
%             sprintf('\t #vehicles in %s: %d\n', obj.platoon.ID,obj.platoon.n),...
%             sprintf('\t max #vehicles in %s: %d\n',platoon.ID,platoon.nmax)
%             ]);
%     end
% else
%     warning([
%         sprintf('Cannot join new platoon.\n'),...
%         sprintf('\t%s is currently in %s mode.\n',obj.ID,obj.q),...
%         sprintf('\tOnly Free, Leader or EmergLeader vehicles can join a platoon.\n')
%         ]);
% end
% 
% end
