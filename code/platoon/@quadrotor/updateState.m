function x1 = updateState(obj, u, x0)
% function x1 = updateState(obj, u, x0)
% Updates state based on control
%
% Inputs:   obj - current quardotor object
%           u   - control (defaults to previous control)
%           x0  - initial state (defaults to current state)
%
% Outputs:  x1  - final state
%
% Mo Chen, 2015-05-24

% If no control is specified, use previous control
if nargin < 2, u = obj.u; end

% If no state is specified, use current state
if nargin < 3, x0 = obj.x; end

x1 = obj.computeState(u, x0);

% Update the state, state history, control, and control history
obj.x = x1;
obj.u = u;

obj.xhist = cat(2, obj.xhist, obj.x);
obj.uhist = cat(2, obj.uhist, obj.u);
end