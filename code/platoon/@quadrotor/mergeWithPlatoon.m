function u = mergeWithPlatoon(obj, p)
% function u = mergeWithPlatoon(obj, p)
%
% Inputs:  obj  - quadrotor objects
%          p    - platoon object to merge with
%
% Output:  u - control signal for the merge
%
% 2015-06-17, Mo

% Check if platoon is full (should never have to check if outside logic is
% correct
if p.n >= p.nmax
    error('Platoon already full!')
end

pdim = obj.pdim;
vdim = obj.vdim;

% Update join list
if isempty(obj.pJoin)
    % If current vehicle is not currently joining a platoon, then mark
    % p as the platoon to join and add to join list
    obj.idx = find(~p.vList, 1, 'first');
    p.vList(obj.idx) = -1;
    obj.pJoin = p;
    
else
    % Otherwise, empty previous marked platoon and remove from other
    % platoon's join list; also mark this platoon for joining
    if obj.pJoin ~= p
        obj.pJoin.vList(obj.idx) = 0;
        obj.idx = find(~p.vList, 1, 'first');
        p.vList(obj.idx) = -1;
        obj.pJoin = p;
    end
end

% Parse target state
x = zeros(4,1);
x(vdim) = [0 0];

% Determine phantom position (First free position)
xPh = p.phantomPosition(obj.idx);
x(pdim) = xPh - p.vehicle(1).x(p.vehicle(1).pdim);

% Time horizon for MPC
tsteps = 5;

if strcmp(obj.q, 'Free') || strcmp(obj.q, 'EmergLeader')
    % If vehicle is free or an emergency leader, then try to join the
    % platoon at the back
    
    % Reachable set from target state
    if isempty(obj.mergePlatoonV)
        [ datax, datay, g1, g2, tau ] = quad2D_joinHighwayPlatoon(x, 0);
        obj.mergePlatoonV.datax = datax;
        obj.mergePlatoonV.datay = datay;
        obj.mergePlatoonV.g1 = g1;
        obj.mergePlatoonV.g2 = g2;
        obj.mergePlatoonV.tau = tau;
        
    else
        datax = obj.mergePlatoonV.datax;
        datay = obj.mergePlatoonV.datay;
        g1 = obj.mergePlatoonV.g1;
        g2 = obj.mergePlatoonV.g2;
        tau = obj.mergePlatoonV.tau;
        
    end
    
    if abs(x-(obj.x-p.vehicle(1).x))<=1.1*[g1.dx;g2.dx]
        % if relative state is within one grid point of target relative
        % state, roughly, then vehicle gets assimilated into platoon
        p.assimVehicle(obj);
        u = obj.followPlatoon;
        
    else
        % if relative state is not close enough, then try to head towards
        % the relative target state
        [valuex, gradx] = recon2x2D(tau, g1, datax, g2, ...
            datay, obj.x-p.vehicle(1).x);
        
        if valuex <= 0
            % if vehicle is in the reachable set of target relative state,
            % use optimal control
            disp('Locked in')
            ux = (gradx(2)>=0)*obj.uMin + (gradx(2)<0)*obj.uMax;
            uy = (gradx(4)>=0)*obj.uMin + (gradx(4)<0)*obj.uMax;
            u = [ux; uy];
            
        else
            % if vehicle is not in the reachable set, move towards target
            % position in a straight line
            disp('Open-loop')
            
            % Path to target
            pathToOther = highway(obj.x(pdim), xPh);
            u = obj.followPath(tsteps, pathToOther);

        end
    end
    
elseif strcmp(obj.q, 'Follower')
    error('Vehicle cannot be a follower!')
    
elseif strcmp(obj.q, 'Leader')
    error('Vehicle cannot a leader!')
    
else
    error('Unknown mode!')
end

end
