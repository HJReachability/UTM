function addHighway(obj, hw)
% function addHighway(obj, hw)
%
% Adds a highway to the list of highways managed by the tfm
%
% Mo Chen, 2015-10-30

if ~isa(hw, 'highway')
  error('Input must be a highway object!')
end

% Add highway to highways list
obj.hws = {hws; hw};

% Add platoons on highway to active agents list
addActiveAgent(hw.ps);

end