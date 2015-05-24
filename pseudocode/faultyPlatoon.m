% Platoon platoon interaction with faulty vehicle
% Need to know which platoon is in front and behind it. Include this \
% information in highway class?

% Platoon check safety

[safe, uSafe, ~, ~] = platoon.isSafe(obj, other);

% If not safe, platoon leader applies uSafe
if ~safe
    platoon.vehicle{1}.u = uSafe;
end


function [safe, uSafe, valuex, valueCx] = isSafe(obj, other, safeV, t)
% Inputs:  obj   - this vehicle
%          other - other vehicle
%          t     - time horizon

% If other is in front of obj
if obj.FP == other
    obj.vehicle{1}.isSafe(other.vehicle{n}, safeV) 
elseif obj.BP == other
    obj.vehicle{n}.isSafe(other.vehicle{1}, safeV)
else
    % error('Only check platoon safety with nearby platoons')??
end
end