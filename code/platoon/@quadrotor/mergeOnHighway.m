function u = mergeOnHighway(obj, hw, target)
% function u = mergeOnHighway(obj, hw, target)
%
% Inputs:  target  - target position on highway (2D vector or scalar
%                    between 0 and 1
%          hw      - highway object to merge onto
%
% Output:  u - control signal to merge onto highway
%
% 2015-06-17, Mo Chen

% Parse target state
x = zeros(1,4);
if numel(target) == 1
  s = target;
  s = min(1,s);
  s = max(0,s);
  x(obj.pdim) = hw.fn(s);
  
elseif numel(target) == 2
  x(obj.pdim) = target;
  
else
  error('Invalid target!')
end

% Target velocity (should be velocity along the highway)
x(obj.vdim) = hw.speed * hw.ds;


% Time horizon for MPC
tsteps = 5;

switch obj.q
  case 'Free'
    % Reachable set from target state; compute it if it hasn't been
    % computed yet; otherwise simply access it
    %         if isempty(obj.mergeHighwayV)
    %             [ datax, datay, g, tau ] = quad2D_joinHighway(x, 0);
    %             obj.mergeHighwayV.datax = datax;
    %             obj.mergeHighwayV.datay = datay;
    %             obj.mergeHighwayV.g = g;
    %             obj.mergeHighwayV.tau = tau;
    %             obj.mergePlatoonV = [];
    %         else
    %             datax = obj.mergeHighwayV.datax;
    %             datay = obj.mergeHighwayV.datay;
    %             g = obj.mergeHighwayV.g;
    %             tau = obj.mergeHighwayV.tau;
    %         end
    %
    
    % state on liveness reachable set grid
    liveV = hw.liveV;
    
    x_liveV(obj.pdim)=obj.x(obj.pdim)-x(obj.pdim)';
    x_liveV(obj.vdim)=obj.x(obj.vdim);
    
    if iscell(liveV.g)
      tol = 1.1 * [liveV.g{1}.dx; liveV.g{2}.dx];
    elseif isstruct(liveV.g)
      tol = 1.1 * liveV.g.dx;
    else
      error('liveV.g must be a cell structure or a struct!')
    end
    
    err=obj.x-x';
    
    if abs(err)<=tol
      % Perform merging maneuver until obj becomes leader
      % If we're close to the target set, form a platoon and become a leader
      obj.p = platoon(obj, hw);  % Create platoon
      u = obj.followPath(tsteps, hw);
      
    else % otherwise head towards target state
      % the liveness reachable set will either be expressed by 2x2D value 
      % functions or a 4D value function
      if iscell(liveV.g)
        % If liveV.g is a cell structure, then it must be a cell structure
        % of two elements, each containing the grid corresponding to a 2D
        % reachable set
        grids = liveV.g;
        datas = liveV.data;
        tau = liveV.tau;
        
        TD_out_x = recon2x2D(tau, grids, datas, x_liveV);
        valuex = TD_out_x.value;
        gradx = TD_out_x.grad;
        
        % Check to see if we're inside reachable set; a value of <= 0
        % incidcates that we're within 5 seconds to getting to the target
        inside_RS = valuex<=0; 
      else
        % if liveV.g is not a cell structure, then it must be a struct
        % representing the grid structure for a 4D reachable set
        g = liveV.g;
        data = liveV.data;
        grad = liveV.grad;
        
        valuex = eval_u(g, data, x_liveV);
        gradx = calculateCostate(g, grad, x_liveV);
        
        % Check to see if we're within 5 seconds to getting to the target
        inside_RS = valuex<=5;
      end
      
      %         % Perform merging maneuver until obj becomes leader
      %         % If we're close to the target set, form a platoon and become a leader
      %         if abs(x'-obj.x)<=1.1*[g.dx;g.dx]
      %             obj.p = platoon(obj, hw);  % Create platoon
      %             u = obj.followPath(tsteps, hw);
      %
      %         else
      %             % Otherwise, compute V(t,obj.x) at the first t such at V(t,obj.x)<=0
      %            [valuex, gradx] = recon2x2D(tau, g, datax, g, datay, obj.x);
      
      if inside_RS % If we're inside reachable set, start merging
        disp('Locked-in')
        ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
        uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
        u = [ux; uy];
      else % Otherwise, simply take a straight line to the target
        
        disp('Open-loop')
        % Path to target         
        pathToTarget = linpath(obj.x(obj.pdim), target);
        
        u = obj.followPath(tsteps, pathToTarget);
      end
    end
    
  case 'Leader'
    error('Vehicle cannot be a leader!')
    % Unless we're joining another highway... need to implement this
    
  case 'Follower'
    error('Vehicle cannot be a follower!')
    
  otherwise
    error('Unknown mode!')
end

end