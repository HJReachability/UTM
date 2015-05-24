function u = mergeWithQuadrotor(obj, other, highway, v)
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

pdim = obj.pdim;
vdim = obj.vdim;

% Parse target state
x = zeros(4,1);
x(vdim) = [0 0];

% Direction of highway
ds = highway(1) - highway(0);
vxh = v*ds(1)/norm(ds);
vyh = v*ds(2)/norm(ds);

x(pdim) = other.platoon.phantomPosition(highway, obj.idx) - other.x(pdim);
% keyboard
% Time horizon for MPC
tsteps = 5;

if strcmp(obj.q, 'Free')
    % Reachable set from target state
    if isempty(obj.mergePlatoonV)
        [ datax, datay, g1, g2, tau ] = quad2D_joinHighwayPlatoon(x, 0);
        obj.mergePlatoonV.datax = datax;
        obj.mergePlatoonV.datay = datay;
        obj.mergePlatoonV.g1 = g1;
        obj.mergePlatoonV.g2 = g2;
        obj.mergePlatoonV.tau = tau;
        obj.mergeHighwayV = [];
    else
        datax = obj.mergePlatoonV.datax;
        datay = obj.mergePlatoonV.datay;
        g1 = obj.mergePlatoonV.g1;
        g2 = obj.mergePlatoonV.g2;
        tau = obj.mergePlatoonV.tau;
    end
    
    if abs(x-(obj.x-other.x))<=1.1*[g1.dx;g2.dx]
        %             obj.q = 'Follower';
        %             obj.mergePlatoonV = [];
        %
        %             % Update pointers of current vehicle
        %             obj.platoon = other.platoon;           % Pointer to platoon
        %             obj.FQ = other;                        % Vehicle in front
        %             obj.BQ = obj;                          % Vehicle behind
        %             obj.Leader = other.platoon.vehicle{1}; % Leader of platoon
        %             obj.idx = obj.platoon.n + 1;           % Platoon index
        %
        %             % Update pointers of other vehicle and of the platoon
        %             other.BQ = obj;                  % Other vehicle's previous vehicle
        %             obj.platoon.n = obj.platoon.n+1; % Number of vehicles in platoon
        %             % Add current vehicle to platoon
        %             obj.platoon.vehicle{obj.idx} = obj;
        obj.joinPlatoon(other.platoon);
        u = obj.followPlatoon(highway);
    else
        [valuex, gradx, ~, ~, ~, ind] = recon2x2D(tau, g1, datax, g2, datay, obj.x-other.x);
        %             keyboard
        [tau(ind) valuex]
        if valuex <= 0
            disp('Locked in')
            %                 keyboard
            ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
            uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
            u = [ux; uy];
        else
            disp('Open-loop')
            % Path to target
            xPhantom = other.platoon.phantomPosition(highway, obj.idx);
            
            pathToOther = @(s) [(1-s) * obj.x(pdim(1)) + ...
                s     * xPhantom(1); ...
                (1-s) * obj.x(pdim(2))+ ...
                s     * xPhantom(2)];
            
            ds = pathToOther([0 1]);
            vxp = v*ds(1)/norm(ds);
            vyp = v*ds(2)/norm(ds);
            vx = 0.75*vxp + 0.75*vxh;
            vy = 0.75*vyp + 0.75*vyh;
            %                 u = obj.followPath(tsteps, pathToOther, [vx; vy]);
            u = obj.followPath(tsteps, pathToOther, v);
            %                 pathToOtherd = pathToOther([0 1]);
            %                 hpto = plot(pathToOtherd(1,:), pathToOtherd(2,:), 'k--');
        end
    end
    
elseif strcmp(obj.q, 'EmergLeader')
    % Reachable set from target state
    if isempty(obj.mergePlatoonV)
        [ datax, datay, g1, g2, tau ] = quad2D_joinHighwayPlatoon(x, 0);
        obj.mergePlatoonV.datax = datax;
        obj.mergePlatoonV.datay = datay;
        obj.mergePlatoonV.g1 = g1;
        obj.mergePlatoonV.g2 = g2;
        obj.mergePlatoonV.tau = tau;
        obj.mergeHighwayV = [];
    else
        datax = obj.mergePlatoonV.datax;
        datay = obj.mergePlatoonV.datay;
        g1 = obj.mergePlatoonV.g1;
        g2 = obj.mergePlatoonV.g2;
        tau = obj.mergePlatoonV.tau;
    end
    
    % If already near target (within 1.1 grid points), then simply
    % follow highway; otherwise, get to the highway
    if abs(x-(obj.x-other.x))<=1.1*[g1.dx;g2.dx]
        obj.joinPlatoon(other.platoon);
        u = obj.followPlatoon(highway);
    else
        % Check to see if we're in reachable set
        [valuex, gradx, ~, ~, ~, ind] = recon2x2D(tau, g1, datax, g2, datay, obj.x-other.x);
        [tau(ind) valuex]
        if valuex <= 0 % If in reachable set, follow optimal control
            disp('Locked in')
            ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
            uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
            u = [ux; uy];
        else           % Otherwise, simply head to target in a straight line
            disp('Open-loop')
            % Path to target
            xPhantom = other.platoon.phantomPosition(highway, obj.ID);
            
            pathToOther = @(s) [(1-s) * obj.x(pdim(1)) + ...
                s     * xPhantom(1); ...
                (1-s) * obj.x(pdim(2))+ ...
                s     * xPhantom(2)];
            
            %                 ds = pathToOther([0 1]);
            %                 vxp = v*ds(1)/norm(ds);
            %                 vyp = v*ds(2)/norm(ds);
            %                 vx = 0.5*vxp + 0.5*vxh;
            %                 vy = 0.5*vyp + 0.5*vyh;
            %                 u = obj.followPath(tsteps, pathToOther, [vx; vy]);
            u = obj.followPath(tsteps, pathToOther, v);
            %                 pathToOtherd = pathToOther([0 1]);
            %                 hpto = plot(pathToOtherd(1,:), pathToOtherd(2,:), 'k--');
            obj.q = 'Follower';
        end
    end
    
elseif strcmp(obj.q, 'Follower')
    warning('Vehicle is already a follower!')
    u = obj.followPlatoon(highway);
    
elseif strcmp(obj.q, 'Leader')
    error('Vehicle is already a leader!')
    
    
else
    error('Unknown mode!')
end
end
