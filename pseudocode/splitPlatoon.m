function splitPlatoon(platoon, reach)


%% change status

self.mode = leader
self.platoon_index = 1

self.FQ = []
self.Leader = []


%% message vehicle behind

if ~isempty(self.BQ)
    self.BQ.flag_newLeader = 1
    self.BQ.msg.Leader = self  % send proper pointer
    self.BQ.msg.platoon_index = self.platoon_index + 1
end


%% message vehicle in front
if ~isempty(self.FQ)
    self.FQ.flag_newBQ = 1
    self.FQ.msg.newBQ = []
end