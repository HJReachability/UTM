function dOpt = optDstb(obj, t, y, deriv, ~, ~)
% dOpt = optDstb(obj, t, y, deriv, ~, ~)

if ~iscell(y)
  deriv = num2cell(y);
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

% Minimize Hamiltonian
dOpt = (deriv{3}>=0)*(-obj.bMax) + (deriv{3}<0)*obj.bMax;


end