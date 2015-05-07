function simulateTwoQuadrotors ()

close all

tEnd = 4;                  % End of simulation time
dt = 0.1;                   % Sampling time
pdim = [1,3];               % Index of position states
gridSize = 15;             % Size of grid
tStep = 10;

% Get reachability information
reachInfo = generateReachInfo();

% put some quadrotors in a horizontal line and create a platoon
Q = quadrotor(1, dt, [gridSize/2;0;gridSize/2;0], reachInfo);

% intruding quadrotor
intruderX0 = [gridSize;0];     % Initial position
intruderXt = [gridSize/2;gridSize/2];       % Final position
I = quadrotor(0, dt, [intruderX0(1);0;intruderX0(2);0]);
intruderPath = @(s)rpath(s,intruderX0,intruderXt);

tNow = 0;

% Plot quadrotors
h1t = plot(Q.xhist(1,:), Q.xhist(3,:), 'b-'); hold on %Path
h2t = plot(I.xhist(1,:), I.xhist(3,:), 'r-');
h1p = plot(Q.x(1), Q.x(3), 'bo'); % Present position
h2p = plot(I.x(1), I.x(3), 'ko');

axis([0,gridSize, 0, gridSize]);
title(sprintf('t = %.1f',tNow));
drawnow;

while tNow < tEnd
    
    % For each quadrotor in platoon check if it's safe against I
    [safe, uSafe, valuex] = Q.isSafe(I, I.tauInt);
    if safe
        Q.u = zeros(2,1);
    else
        Q.u = uSafe;
    end
    u = I.followPath(tStep, intruderPath, I.vMax);
    I.u = u(:,1);
    
    % Update states for all quadrotors
    I.updateState(I.u);
    Q.updateState(Q.u)
    
    tNow = tNow + dt; sprintf('tNow = %d', tNow);
    
    h1t.XData = Q.xhist(1,:);
    h1t.YData = Q.xhist(3,:);
    h1p.XData = Q.x(1);
    h1p.YData = Q.x(3);
    
    h2t.XData = I.xhist(1,:);
    h2t.YData = I.xhist(3,:);
    h2p.XData = I.x(1);
    h2p.YData = I.x(3);
    
    drawnow;
    title(sprintf('t = %.1f. safeV = %.2f, safe = %0.1f',tNow, valuex, safe));

end



