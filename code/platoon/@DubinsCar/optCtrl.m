function uOpt = optCtrl(obj, t, y, deriv, uMode, MIEdims)
% [uOpt, dOpt] = optCtrl(obj, t, y, deriv, uMode, dMode, MIEdims)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if nargin < 6
  MIEdims = 0;
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
if strcmp(uMode, 'max')
  uOpt = (deriv{3-MIEdims}>=0)*obj.wMax - (deriv{3-MIEdims}<0)*obj.wMax;
elseif strcmp(uMode, 'min')
  uOpt = -(deriv{3-MIEdims}>=0)*obj.wMax + (deriv{3-MIEdims}<0)*obj.wMax;
else
  error('Unknown uMode!')
end

end