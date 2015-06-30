function p = popPlatoon(hw, leaderPos, leaderVel, numVehicles, ID1)
% function p = popPlatoon(hw, leaderPos, leaderVel, numVehicles, ID1)
%
% Populate a platoon on a highway hw with numVehicles vehicles; the leader
% has position leaderPos, velocity leaderVel and ID ID1.
%
% ID1 - starting ID
%

if nargin<3, leaderVel = [0;0]; end
if nargin<4, numVehicles = 1; end
if nargin<5,         ID1 = 1; end

% Hard code time discretization for now
dt = 0.1;

switch numel(leaderPos)
    case 2
        % If input is a 2D vector, find nearest position on the highway
        [~, ~, x1] = hw.highwayPos(leaderPos);
        
    case 1
        % If input position is a scalar, interpret as highway parameter and
        % project it onto the interval [0, 1]
        s = leaderPos;
        s = min(s,1);
        s = max(s,0);
        
        x1 = hw.fn(s);
    otherwise
        error('Invalid leader position!')
end

if numel(leaderPos) ~= 2
    error('Invalid leader velocity!')
end

% Create first vehicle and platoon
reachInfo = generateReachInfo();

x = zeros(4,1);
qr = quadrotor(ID1, dt, x, reachInfo);
qr.x(qr.pdim) = x1;
qr.x(qr.vdim) = leaderVel;

p = platoon(qr, hw); % Using default nmax and followTime

for i = 2:numVehicles
    ID = ID1 + i - 1;
    
    p.addVehicle(reachInfo, ID);
end


end