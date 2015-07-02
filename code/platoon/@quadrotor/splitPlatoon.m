function p = splitPlatoon(obj)
% Split current platoon in two and become Leader of the trailing platoon
% 2015-06-30 Qie Hu


if ~strcmp(obj.q,'Follower')
    warning([
        sprintf('Cannot split from platoon.\n'),...
        sprintf('\t%s is currently in %s mode.\n',obj.ID,obj.q),...
        sprintf('\tOnly Follower vehicles can split form a platoon.\n')
        ]);
    p = [];
    return
end

% Platoon before splitting
% idx of vehicle in the original platoon before splitting 
orig_p = obj.p;
orig_q_idx = obj.idx;

% Create new platoon (same followTime as existing platoon) with single
% vehicle/leader
% Platoon pointers initialized to itself
% Vehicle pointers initialized to itself
p = platoon(obj, obj.p.hw, obj.p.nmax - (obj.idx-1), obj.p.followTime);
% p = platoon(obj, obj.p.hw);

% If there was a platoon behind,
% update pointers for new platoon and the platoon behind
if orig_p.BP ~= orig_p   
    p.BP            = orig_p.BP; 
    orig_p.BP.FP    = p; 
end

% Update pointer for new platoon and the original platoon in front
orig_p.BP           = p;
p.FP                = orig_p;

% Split all lists and update info
p.vehicle                   = orig_p.vehicle(orig_q_idx:end);
p.vList                     = orig_p.vList(orig_q_idx:end);
% p.IDvehicle               = orig_p.IDvehicle(orig_q_idx:end);
p.n                         = orig_p.n - (orig_q_idx - 1);
orig_p.vehicle              = orig_p.vehicle(1:orig_q_idx-1);
% orig_p.IDvehicle          = orig_p.IDvehicle(1:orig_q_idx-1);
orig_p.vList(orig_q_idx:end)= 0;
orig_p.n                    = orig_q_idx - 1;

% Update vehicle pointers
orig_p.vehicle(orig_p.n).BQ = orig_p.vehicle(orig_p.n);
if p.n > 1
    obj.BQ                  = p.vehicle(2);
end

% Update follower vehicles in new platoon
obj.q = 'EmergLeader';
for i = 2:p.n
    p.vehicle(i).p          = p;
    p.vehicle(i).idx        = i;
    p.vehicle(i).Leader     = p.vehicle(1);
end

end

