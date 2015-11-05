function [safe, uSafe] = checkAASafety(obj)
% function [safe, uSafe] = checkAASafety(obj)
%
% Checks the safety of every active agent with respect to all other active
% agents
%
% Input:   obj   - tfm object
% outputs: safe  - boolean vector; row i specifies whether agent i is safe
%                  with respect to all other agents
%          uSafe - cell vector of safety-preserving controls
%
% Mo Chen, 2015-11-04

% uSafe needs to be cell because controls may have different dimensions in
% different agents
safe = zeros(length(obj.aas));
uSafe = cell(length(obj.aas), 1); 

% Go through every pair of agents and return safety indicator and control
for i = 1:length(obj.aas)
  % Get safety indicator controller with respect to all other vehicles
  uSafei = cell(1, length(obj.aas));
  for j = 1:length(obj.aas)
    [safe(i,j), uSafei{j}] = obj.checkPWSafety(i, j);
  end
  
  % There should not be more than one conflict
  if nnz(~safe(i,:)) > 1
    error('More than one conflict detected!')
  end
  
  % Update safe-preserving control for agent i
  if nnz(~safe(i,:)) == 1
    uSafe{i} = uSafei{safe(i,:) == 0};
  else
    uSafe{i} = [];
  end
end

safe = prod(safe,2);
end