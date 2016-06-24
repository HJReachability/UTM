function dOpt = optDstb(obj, t, y, deriv, dMode, ~)
% dOpt = optDstb(obj, t, y, deriv, dMode, ~)

%% Input processing
if nargin < 5
  dMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal distrurbance
if strcmp(dMode, 'max')
  dOpt{1} = obj.dMax(1) * deriv{1} ./ sqrt(deriv{1}.^2 + deriv{2}.^2);
  dOpt{2} = obj.dMax(1) * deriv{2} ./ sqrt(deriv{1}.^2 + deriv{2}.^2);
  dOpt{3} = (deriv{3} >= 0) * obj.dMax(2) - (deriv{3} < 0) * obj.dMax(2);
elseif strcmp(dMode, 'min')
  dOpt{1} = -obj.dMax(1) * deriv{1} ./ sqrt(deriv{1}.^2 + deriv{2}.^2);
  dOpt{2} = -obj.dMax(1) * deriv{2} ./ sqrt(deriv{1}.^2 + deriv{2}.^2);
  dOpt{3} = -(deriv{3} >= 0) * obj.dMax(2) + (deriv{3} < 0) * obj.dMax(2);  
else
  error('Unknown dMode!')
end

end
