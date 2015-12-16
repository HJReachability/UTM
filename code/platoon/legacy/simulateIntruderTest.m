function simulateIntruder()

cvx_quiet true


tEnd = 20;                  % End of simulation time
dt = 0.1;                   % Sampling time
numQ = 6;                  % Number of quadrotors in platoon
tFollow = 2;                % Follow time between quadrotors inside platoon
gridSize = 60;             % Size of grid


% Get reachability information
[reachInfo, safeV] = generateReachInfo();

% intruding quadrotor
intruderX0 = [35;20];       % Initial position
intruderXt = [0;gridSize];       % Final position
Intruder = quadrotor(0, dt, [intruderX0(1);-3;intruderX0(2);3]);


% Define paths
highwayPath = @(s)rpath(s,[0;gridSize/2],[gridSize;gridSize/2]);
intruderPath = @(s)rpath(s,intruderX0,intruderXt);


% put some quadrotors in a horizontal line and create a platoon
quadrotors = cell({});
quadrotors{1} = quadrotor(1, dt, [25;4;gridSize/2;0], reachInfo);
platoons{1} = quadrotors{1}.newPlatoon(numQ, tFollow); 
for i = 2:numQ
    quadrotors{i} = quadrotor(i, dt, [0;4;0;0], reachInfo);
    quadrotors{i}.joinPlatoon(platoons{1});
    quadrotors{i}.x(quadrotors{i}.pdim) = quadrotors{i}.phantomPosition(highwayPath);
    quadrotors{i}.xhist(:,1) = quadrotors{i}.x;
end


tNow = 0;
tSteps = 5;

% plotQuadrotors
valSafe = zeros(numQ);
fig = figure; %hts = cell(1,numQ); hps = cell(1,numQ);
drawHighway(fig,highwayPath,gridSize,'black');
for i = 1:numQ
    [hts{i}, hps{i}] = drawQuadrotor(fig,[],[],quadrotors{i});
end
[hti, hpi] = drawQuadrotor(fig,[],[],Intruder);

% quadrotors{i}.plotSafeV(Intruder,'r', quadrotors{i}.tauExt);
axis([0,gridSize, 0, gridSize]);
title(sprintf('t = %.1f',tNow));
drawnow;

while tNow < tEnd
    
    
    % For each quadrotor in platoon check if it's safe against Intruder
    for i = 1:length(quadrotors)
        [safe, uSafe, valSafe(i)] = quadrotors{i}.isSafe(Intruder, safeV, quadrotors{i}.tauExt);
        
        if safe
            
            % Intruder outside safe distance
            if strcmp(quadrotors{i}.q, 'Leader')
                quadrotors{i}.u = quadrotors{i}.followPath(tSteps, highwayPath, quadrotors{i}.vMax);
            elseif strcmp(quadrotors{i}.q, 'EmergLeader')
                quadrotors{i}.q = 'Leader';
                quadrotors{i}.u = quadrotors{i}.followPath(tSteps, highwayPath, quadrotors{i}.vMax);
            else
                quadrotors{i}.u = quadrotors{i}.followPlatoon(safeV, highwayPath);
            end
            
        else
            
            % Intruder inside safe distance
            % If quadrotor is a Leader, it stays a Leader 
            % If quadrotor is a Follower, it becomes an EmergLeader and
            % splits platoon
            if strcmp(quadrotors{i}.q, 'Follower')
                quadrotors{i}.splitPlatoon();
            end
            % from this point, quadrotors{i} is EmergLeader
            quadrotors{i}.u = uSafe;
        end
    end
    
    u = Intruder.followPath(tSteps, intruderPath, Intruder.vMax);
    Intruder.u = u(:,1);
    
    
    % Update states for all quadrotors
    Intruder.updateState(Intruder.u);
    for i = 1:length(quadrotors)
        quadrotors{i}.updateState(quadrotors{i}.u) % can be noisy
    end
    
    tNow = tNow + dt; sprintf('tNow = %d', tNow);
    
    % plotQuadrotors
    [~, unsafestQIdx] = min(valSafe);
    for i = 1:numQ
        drawQuadrotor(fig,hts{i},hps{i},quadrotors{i});
    end
    drawQuadrotor(fig,hti,hpi,Intruder);    
    title(sprintf('t = %.1f',tNow));
    drawnow;
    
    
end


end



