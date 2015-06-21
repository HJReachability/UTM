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
        if isempty(obj.mergeHighwayV)
            [ datax, datay, g, tau ] = quad2D_joinHighway(x, 0);
            obj.mergeHighwayV.datax = datax;
            obj.mergeHighwayV.datay = datay;
            obj.mergeHighwayV.g = g;
            obj.mergeHighwayV.tau = tau;
            obj.mergePlatoonV = [];
        else
            datax = obj.mergeHighwayV.datax;
            datay = obj.mergeHighwayV.datay;
            g = obj.mergeHighwayV.g;
            tau = obj.mergeHighwayV.tau;
        end
        
        % Perform merging maneuver until obj becomes leader
        % If we're close to the target set, form a platoon and become a leader
        if abs(x'-obj.x)<=1.1*[g.dx;g.dx]
            obj.p = platoon(obj, hw);  % Create platoon
            u = obj.followPath(tsteps, hw);
            
        else
            % Otherwise, compute V(t,obj.x) at the first t such at V(t,obj.x)<=0
            [valuex, gradx] = recon2x2D(tau, g, datax, g, datay, obj.x);
            
            if valuex <= 0 % If we're inside reachable set, start merging
                disp('Locked-in')
                ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
                uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
                u = [ux; uy];
                
            else           % Otherwise, simply take a straight line to the target
                disp('Open-loop')
                % Path to target (for now written as a highway object, which isn't really
                % "correct" in principle)
                pathToTarget = highway(obj.x(obj.pdim), target);
                
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