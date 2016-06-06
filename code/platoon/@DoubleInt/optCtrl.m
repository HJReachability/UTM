function uOpt = optCtrl(obj, t, y, deriv, uMode, MIEdims)
% uOpt = optCtrl(obj, t, y, deriv, uMode, MIEdims)

if nargin < 5
  uMode = 'min';
end

if nargin < 6
  MIEdims = 0;
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

if strcmp(uMode, 'max')
  uOpt = (deriv{2-MIEdims}>=0)*obj.uMax + (deriv{2-MIEdims}<0)*obj.uMin;
elseif strcmp(uMode, 'min')
  uOpt = (deriv{2-MIEdims}>=0)*obj.uMin + (deriv{2-MIEdims}<0)*obj.uMax;
else
  error('Unknown uMode!')
end


end