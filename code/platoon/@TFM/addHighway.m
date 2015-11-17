function addHighway(obj, hw)
% function addHighway(obj, hw)
%
% Adds a highway to the list of highways managed by the tfm
%
% Mo Chen, 2015-10-30

if ~isa(hw, 'Highway')
  error('Input must be a highway object!')
end

% Add highway to highways list
obj.hws{length(obj.hws)+1} = hw;

% Add platoons on highway to active agents list
obj.addActiveAgents(hw.ps);

end