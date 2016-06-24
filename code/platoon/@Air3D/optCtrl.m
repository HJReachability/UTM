function uOpt = optCtrl(obj, t, y, deriv, ~, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode, ~)

if ~iscell(y)
  deriv = num2cell(y);
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

% Determinant for sign of control
det = deriv{1}.*y{2}  - deriv{2}.*y{1} - deriv{3};

% Maximize Hamiltonian
uOpt = (det>=0)*obj.aMax + (det<0)*(-obj.aMax);

end