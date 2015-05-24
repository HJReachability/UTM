function abandonPlatoon(platoon, reach)


%% message vehicle behind

if ~isempty(self.BQ)
    self.BQ.flag_newFQ = 1
    self.BQ.msg.FQ = self.FQ
    self.BQ.msg.platoon_index = self.platoon_index
end


%% message vehicle in front

if ~isempty(self.FQ)
    self.FQ.flag_newBQ = 1
    self.FQ.msg.newBQ = self.BQ
end


%% change status

self.mode = free
self.platoon_index = []


self.FQ = []
self.BQ = []
self.Leader = []