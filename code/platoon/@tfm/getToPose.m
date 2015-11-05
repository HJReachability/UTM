function u = getToPose(obj, vehicle, position, heading, debug)
% function u = getToState(obj, vehicle, target_state)
%
% Computes the control that drives the vehicle to some target state
%
% Mo Chen, 2015-11-03

% Make sure target state is valid
if numel(position) ~= 2
  error('Incorrect number of dimensions in target state!')
end

if ~iscolumn(position)
  position = position';
end

if ~isscalar(heading)
  error('Heading must be a scalar representing an angle!')
end

if nargin < 5
  debug = false;
end

switch class(vehicle)
  case 'Quadrotor'
    % Get state in the target heading frame
    base_pos = rotate2D(vehicle.getPosition - position, -heading);
    base_vel = rotate2D(vehicle.getVelocity, -heading);
    base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];
    
    % Get value function
    valuex = eval_u(obj.qr_create_platoon_V.g, ...
      obj.qr_create_platoon_V.data, base_x);
    
    % Arrived at target
    if valuex <= obj.ttt
      if debug
        disp('Arrived at target')
      end
      u = [];
      return;
    end
    
    % Inside reachable set
    if valuex <= obj.rtt
      if debug
        disp('Inside reachable set')
      end
      vehicle.waypoints = [];
      base_p = calculateCostate(obj.qr_create_platoon_V.g, ...
        obj.qr_create_platoon_V.grad, base_x);
      
      ux = (base_p(2)>=0)*vehicle.uMin + (base_p(2)<0)*vehicle.uMax;
      uy = (base_p(4)>=0)*vehicle.uMin + (base_p(4)<0)*vehicle.uMax;
      u = rotate2D([ux; uy], heading);
      return;
    end
    
    % Outside reachable set
    if debug
      disp('Outside reachable set')
    end
    
    if isempty(vehicle.waypoints)
      vehicle.waypoints = ...
        linpath(vehicle.getPosition, position, obj.hw_speed);
    end

    u = vehicle.followPath(vehicle.waypoints);
    
  otherwise
    error('This function is implemented for the Quadrotor class!')
end



end