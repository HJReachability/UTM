function p = splitPlatoon(obj)
% Split current platoon in two and become Leader of the trailing platoon
if ~strcmp(obj.q,'Follower')
    warning([
        sprintf('Cannot split from platoon.\n'),...
        sprintf('\t%s is currently in %s mode.\n',obj.ID,obj.q),...
        sprintf('\tOnly Follower vehicles can split form a platoon.\n')
        ]);
    p = [];
    return
end
% Create new platoon (max size to allow re-joining, same followTime as existing platoon)
p = platoon(obj, obj.platoon.nmax - (obj.idx-1), obj.platoon.followTime);

% Update platoon pointers
if obj.platoon.BP == obj.platoon,   
    p.BP = p;
else
    p.BP = obj.platoon.BP; 
    obj.platoon.BP.FP = p; 
end
obj.platoon.BP        = p;
p.FP                  = obj.platoon;

% Split all lists and update info
p.vehicle             = obj.platoon.vehicle(obj.idx:end);
p.IDvehicle           = obj.platoon.IDvehicle(obj.idx:end);
p.n                   = obj.platoon.n - (obj.idx - 1);
obj.platoon.vehicle   = obj.platoon.vehicle(1:obj.idx-1);
obj.platoon.IDvehicle = obj.platoon.IDvehicle(1:obj.idx-1);
obj.platoon.n         = obj.idx - 1;

% Update vehicle pointers
obj.platoon.vehicle{obj.platoon.n}.BQ = obj.platoon.vehicle{obj.platoon.n};
obj.FQ                      = obj;

% Update follower vehicles in new trailing platoon
% (after this point the function loses original platoon handle)
obj.q = 'EmergLeader';
for i = 1:p.n
    p.vehicle{i}.platoon = p;
    p.vehicle{i}.idx = i;
    p.vehicle{i}.Leader = p.vehicle{1};
end
end