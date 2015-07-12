function [u1, u] = followPath(obj, tsteps, hw, v)
% function [u1, u] = followPath(obj, tsteps, hw, v)
%
% Computes the next control input to follow the path rpath
%
% Inputs: obj    - vehicle object
%         tsteps - number of time steps to look ahead (time horizon is
%                  tsteps*obj.dt)
%         rpath  - path to follow
%                    should be a function handle that takes s as input and
%                    outputs points on the path rpath(s)
%         v      - speed on the path
%
% Outputs: u1 - first control action from optimization (only use this for
%               MPC)
%          u  - entire control function from optimization
%
% Mo Chen, 2015-05-23

% Find closest point on the path to current position

global K LQROn  %in case we want to use LQR controller
s0 = firstPathPoint(obj, hw.fn);

if nargin < 4
    v = hw.speed;
end

% Reference velocity
if numel(v) == 1  % Convert to velocity along the highway if needed
    vref = v * hw.ds;
else
    vref = v;
end

if ~LQROn   

    % ----- BEGIN CVX -----
    cvx_begin
    variable p(2, tsteps)     % sequence of vehicle positions
    variable v(2, tsteps)     % sequence of vehicle velocities
    variable r(2, tsteps)     % sequence of reference positions
    variable s(1, tsteps)       % sequence of reference path indices
    variable ds(1,tsteps-1)     % sequence of speeds along the path
    variable u(obj.nu, tsteps)  % sequence of controls
    variable x(obj.nx, tsteps)  % sequence of states

    minimize sum(sum((r-p).^2)) + 5*sum(1-s) + sum(sum( (v(1,:)-vref(1)).^2 + (v(2,:)-vref(2)).^2 ))

    subject to
    % First time step
    x(:,1) == obj.computeState(u(:,1), obj.x)   % Dynamics
    p(:,1) == x(obj.pdim,1)                        % Position components
    v(:,1) == x(obj.vdim,1)
    s(1) == s0

    %             All time steps afterwards
    for i = 2:tsteps
        x(:,i) == obj.computeState(u(:,i), x(:,i-1))        % Dynamics
        p(:,i) == x(obj.pdim,i)                                % Position components
        v(:,i) == x(obj.vdim,i)                                % Position components
        s(i) == s(i-1) + ds(i-1)                            % Advanced on path
    end

    r == hw.fn(s)
    obj.uMin <= u <= obj.uMax              % Control bounds
    % obj.vMin <= v <= obj.vMax                 % Velocity bounds
    0 <= s <= 1
    ds >= 0

    cvx_end 
    % ----- END CVX -----
    if any(isnan(u(:))), keyboard; end  %MPC just takes the first control
    u1 = u(:,1);
else
    % ----- BEGIN LQR ---
    sref=s0;
    rref=hw.fn(sref);
    u1=-K*(obj.x-[rref(1);vref(1);rref(2);vref(2)]);
    max(min(u1,obj.uMax),obj.uMin);
    % ----- END LQR -----
end 

end


function s0 = firstPathPoint(obj, rpath)
% function s0 = firstPathPoint(rpath, x)
%
% Computes the parameter s0 on rpath that is closest to the current
% position
%
% Inputs:  obj   - current quadrotor object
%          rpath - a straight line path
%
% Output:  s0    - path parameter: rpath(s0) gives the position closest to
%                  obj.x(p.dim)
%
% Mo Chen, 2015-05-23

x = obj.x;
p = x(obj.pdim); % Position components

N = 1000;
s = linspace(0,1,N);
rpathd = rpath(s);

[~, ipath] = min((rpathd(1,:)-p(1)).^2 + (rpathd(2,:)-p(2)).^2);
s0 = s(ipath);
% if p(2) >= 3, keyboard; end
end