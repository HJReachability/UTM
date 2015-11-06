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
    switch vehicle.live_status
      case 'outRS'
        % Outside reachable set
        valuecx = eval_u(obj.qr_atcV.g, obj.qr_atcV.data, base_x);
      
        % Check if vehicle has arrived; if so, update status and recompute
        % control
        if valuecx <= obj.ttt
          vehicle.live_status = 'arrived';
          vehicle.waypoints = [];          
          u = obj.getToPose(vehicle, position, heading, debug);
          return;
        end
        
        % Check if vehicle is in coarse reachable set; if so, update
        % status and recompute control
        if valuecx <= obj.crtt
          vehicle.live_status = 'inRS';
          vehicle.waypoints = [];          
          u = obj.getToPose(vehicle, position, heading, debug);
          return;
        end
        
        % If vehicle is in fact outside of reachable set, plan a straight
        % path to target position
        if debug
          disp('Outside reachable set')
        end        
        if isempty(vehicle.waypoints)
          vehicle.waypoints = ...
            Linpath(vehicle.getPosition, position, obj.hw_speed);
        end
        
        vehicle.waypoints.lpPlot;
        u = vehicle.followPath(vehicle.waypoints);
        
      case 'arrived'
        
        if debug
          valuecx = eval_u(obj.qr_atcV.g, obj.qr_atcV.data, base_x);
          valuefx = eval_u(obj.qr_atfV.g, obj.qr_atfV.data, base_x);
          disp(['Arrived at target, coarse value = ' ...
            num2str(valuecx) ', fine value = ' num2str(valuefx)])
        end
        u = [];
        
      case 'inRS'
        % Inside reachable set
        valuecx = eval_u(obj.qr_atcV.g, obj.qr_atcV.data, base_x);
        valuefx = eval_u(obj.qr_atfV.g, obj.qr_atfV.data, base_x);
        
        % Check if vehicle has arrived; if so, update status and recompute
        % control
        if valuecx <= obj.ttt
          vehicle.live_status = 'arrived';
          u = obj.getToPose(vehicle, position, heading, debug);
          return;
        end
        
        % Vehicle has in fact not arrived
        if debug
          disp(['Inside reachable set, coarse value = ' ...
            num2str(valuecx) ', fine value = ' num2str(valuefx)])
        end
        
        if valuefx <= obj.frtt
          disp('Using fine value function')
          base_p = ...
            calculateCostate(obj.qr_atfV.g, obj.qr_atfV.grad, base_x);
        else
          base_p = ...
            calculateCostate(obj.qr_atcV.g, obj.qr_atcV.grad, base_x);          
        end
        
        ux = (base_p(2)>=0)*vehicle.uMin + (base_p(2)<0)*vehicle.uMax;
        uy = (base_p(4)>=0)*vehicle.uMin + (base_p(4)<0)*vehicle.uMax;
        u = rotate2D([ux; uy], heading);
        
      otherwise
        error('Unknown status!')
    end
  otherwise
    error('This function is only implemented for the Quadrotor class!')
end



end