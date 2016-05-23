function relStates = getRelStates(obj, others, wrap)
% function relStates = getRelativeStates(obj, others)
% Finds state of other plane objects with respect to one plane
%
% Inputs:
%   obj    - current plane object
%   others - other planes whose relative states relative to obj are to be found; 
%            this should be a n x 1
%   wrap   - set to 'pi' to wrap to [-pi, pi]
%
% Outputs:  
%   relStates - relative state of each other plane
%
% Mahesh Vashishtha, 2015-10-27
% Modified: Mo Chen, 2016-05-22

others = checkVehiclesList(others, 'Plane');
relStates = zeros(3, length(others));

%% Compute unwrapped states
x_this = obj.x;
for i=1:length(others)
  x_other = others{i}.x;

  xr = x_other - x_this;
  
  % rotate position vector so it is correct relative to xe heading
  xr(1:2) = rotate2D(xr(1:2), x_this(3));

  relStates(:,i) = xr;
end

%% Wrap angle
if strcmp(wrap, 'pi')
  relStates(3,:) = wrapToPi(relStates(3,:));
else 
  relStates(3,:) = wrapTo2Pi(relStates(3,:));
end
end