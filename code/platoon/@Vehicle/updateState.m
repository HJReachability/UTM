function x1 = updateState(obj, u, T, x0)
% function x1 = updateState(obj, u, x0, T)
% Updates state based on control
%
% Inputs:   obj - current quardotor object
%           u   - control (defaults to previous control)
%           T   - duration to hold control
%           x0  - initial state (defaults to current state)
%
% Outputs:  x1  - final state
%
% Mo Chen, 2015-05-24

% If no control is specified, use previous control
if nargin < 2
  u = obj.u;
end

% If no state is specified, use current state
if nargin < 4
  x0 = obj.x;
end

% Do nothing if control is empty
if isempty(u)
  x1 = x0;
  return;
end

% Make sure control input is valid
if ~isnumeric(u)
  error('Control must be numeric!')
end

if numel(u) ~= obj.nu
  error(['Control input must have ' num2str(obj.nu) ' dimensions!'])
end

% Convert control to column vector if needed
if ~iscolumn(u)
  u = u';
end

[~, x] = ode113(@(t,x) obj.dynamics(t, x, u), [0 T], x0);

% Update the state, state history, control, and control history
x1 = x(end, :)';
obj.x = x1;
obj.u = u;

obj.xhist = cat(2, obj.xhist, x1);
obj.uhist = cat(2, obj.uhist, u);
end