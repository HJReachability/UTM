function u = mergeOnHighway(obj, highway, target, v)
% function u = mergeOnHighway(x, v, highway)
%
% Inputs:  target  - target position on highway (row vector)
%          v       - target speed on highway
%          highway - highway to merge onto
%                    should be a function handle that takes s as input and
%                    outputs points on the path rpath(s); returns a column
%                    vector
%
% Output: u - control signal to merge onto highway
%
% Mar. 9, 2015, Mo

% Parse target state
x = zeros(1,4);
x([1 3]) = target;

% Direction of highway
ds = highway(1) - highway(0);
x(2) = v*ds(1)/norm(ds);
x(4) = v*ds(2)/norm(ds);

% Path to target
% Input: s, output: 2x2 matrix; rows are endpoints
pathToTarget = @(s) [(1-s)*obj.x(1)+s*target(1); (1-s)*obj.x(3)+s*target(2)];

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
            obj.q = 'Leader';           % Change mode
            obj.mergeHighwayV = [];            % Delete merge value function
            
            obj.platoon = platoon(obj); % Create platoon
            obj.FQ = obj;               % Initialize quadrotor in front
            obj.Leader = obj;           % Leader pointer
            obj.idx = 1;
            
            u = obj.followPath(tsteps, highway, v);
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
                u = obj.followPath(tsteps, pathToTarget, v);
            end
        end

    case 'Leader'
        warning('Vehicle is already a leader!')
        u = obj.followPath(tsteps, highway, v);
        
    case 'Follower'
        error('Vehicle is already a follower!')
        
    otherwise
        error('Unknown mode!')
end

end