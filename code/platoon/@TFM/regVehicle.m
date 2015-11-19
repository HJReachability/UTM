function regVehicle(obj, agent)
% function regVehicle(obj, agents)
%
% Adds a list of agents to the active list
%
% Mo Chen 2015-11-03
% Modified: Mo Chen, 2015-11-19

% If adding an empty set, do nothing
if isempty(agent)
  return
end

% Check vehicle type
if ~isa(agent, 'Vehicle')
  error('Agent must be a Vehicle!')
end

% Expand cell array and assign an ID to the agent
obj.aas{length(obj.aas) + 1, 1} = agent;
agent.ID = length(obj.aas);
end