function simulateIntruder2()

% Simulates a platoon behavior with intruder
% Same as simulateIntruder(), but plots safe sets for different targets for
% the same Quadrotor (evador). For paper purposes.

cvx_quiet true
warning('off','all')


tEnd = 5;                  % End of simulation time
dt = 0.1;                  % Sampling time
t = 0:dt:tEnd;             % Time horizon
tSteps = 5;
numQ = 4;                  % Number of quadrotors in platoon
gridSize = 70;
v = 3.5;                   % Target highway speed
p = 1;

% Set up variables for plotting snap shots
tplot = [0.6 2.0 3.0 3.5];
spC = 2;
spR = 2;
plotnum = 0;


% Get reachability information
[reachInfo, safeV] = generateReachInfo();

% intruding quadrotor
intruderX0 = [40;30];       % Initial position
intruderXt = [10;20];       % Final position
Intruder = quadrotor(0, dt, [intruderX0(1);-4;intruderX0(2);-2]);


% Define paths
highway = @(s)rpath(s,[0;0],[gridSize;gridSize]);
intruderPath = @(s)rpath(s,intruderX0,intruderXt);


% put some quadrotors in a horizontal line and create a platoon
quadrotors = cell({});
quadrotors{1} = quadrotor(1, dt, [23;4;23;4], reachInfo);
quadrotors{1}.newPlatoon(numQ); 
for i = 2:numQ
    quadrotors{i} = quadrotor(i, dt, [0;4;0;4], reachInfo);
    quadrotors{i}.joinPlatoon(quadrotors{1}.platoon);
    quadrotors{i}.x(quadrotors{i}.pdim) = quadrotors{i}.platoon.phantomPosition(highway, quadrotors{i}.idx);
    quadrotors{i}.xhist(:,1) = quadrotors{i}.x;
end



% Plot initial conditions
f1 = figure; 
f2 = figure;
drawHighway(f2,highway,gridSize); hold on; axis([0,gridSize, 0, gridSize]); 
Intruder.plotPosition('red');
colors = {'k', 'b', [0 0.5 0], [1 0 1]};
for i = 1:numQ
    quadrotors{i}.plotPosition(colors{i});
end
hsafeVI = quadrotors{2}.plotSafeV_2(Intruder, safeV);hsafeVI.LineColor = 'r';
hsafeVFQ = quadrotors{2}.plotSafeV_2(quadrotors{1}, safeV);hsafeVFQ.LineColor = 'k';
hsafeVBQ = quadrotors{2}.plotSafeV_2(quadrotors{3}, safeV);hsafeVBQ.LineColor = [0 0.5 0];
axis([0,gridSize, 0, gridSize]); title(sprintf('t = %.1f',t(1)));%box on
drawnow;



for k = 2:length(t)
    
    fprintf('t = %.1d \n', t(k))
    abandonedQ = [];
    
    for i = 1:length(quadrotors)
        
        valC = zeros(1,3);
        
        fprintf('Q%.0d is %s, in platoon %.0d \n', quadrotors{i}.ID, quadrotors{i}.q, quadrotors{i}.platoon.ID)
        % ------------- Check Safety ------------- %
        
        % Check safety w.r.t. Intruder
        [quadrotors{i}.safeI, uSafeI, ~, valC(1)] = quadrotors{i}.isSafe(Intruder, safeV);
        quadrotors{i}.safeIhist = cat(2, quadrotors{i}.safeIhist, quadrotors{i}.safeI);
        
        
        % Check safety w.r.t. FQ
        if strcmp(quadrotors{i}.q, 'Leader') || strcmp(quadrotors{i}.q, 'EmergLeader')
            % If it's a Leader or EmergLeader, check w.r.t to trailing
            % vehicle of front platoon
            if quadrotors{i}.platoon.FP == quadrotors{i}.platoon % If no platoon in front, set to safe
                quadrotors{i}.safeFQ = true;
            else
                [quadrotors{i}.safeFQ, uSafeFQ, ~, valC(2)] = ...
                    quadrotors{i}.isSafe(quadrotors{i}.platoon.FP.vehicle{quadrotors{i}.platoon.FP.n}, safeV);
            end
        else % Follower
            [quadrotors{i}.safeFQ, uSafeFQ, ~, valC(2)] = quadrotors{i}.isSafe(quadrotors{i}.FQ, safeV);
        end
        quadrotors{i}.safeFQhist = cat(2, quadrotors{i}.safeFQhist, quadrotors{i}.safeFQ);

        
        % Check safety w.r.t. BQ
        if quadrotors{i}.idx == quadrotors{i}.platoon.n 
%             % Trailing quadrotor, check w.r.t. leader of back platoon
%             BP = quadrotors{i}.platoon.BP;
%             if BP == quadrotors{i}.platoon % If no platoon at back, set to safe
                quadrotors{i}.safeBQ = true;
%             else
%                 [quadrotors{i}.safeBQ, uSafeBQ] = quadrotors{i}.isSafe(BP.vehicle{1}, safeV);
%             end
        else
            [quadrotors{i}.safeBQ, uSafeBQ, ~, valC(3)] = quadrotors{i}.isSafe(quadrotors{i}.BQ, safeV);
        end
        quadrotors{i}.safeBQhist = cat(2, quadrotors{i}.safeBQhist, quadrotors{i}.safeBQ);

        % Total number of unsafe targets
        numUnsafeTargets = ~quadrotors{i}.safeI + ~quadrotors{i}.safeFQ + ~quadrotors{i}.safeBQ;
        
        
        
        % ------------- Compute Control ------------- %
        
        if numUnsafeTargets == 0     
            %  Safe w.r.t. Intruder, FQ & BQ, can do anything
            if strcmp(quadrotors{i}.q, 'Leader')
                quadrotors{i}.u = quadrotors{i}.followPath(tSteps, highway, quadrotors{i}.vMax);
            elseif strcmp(quadrotors{i}.q, 'EmergLeader') 
                % Join platoon in front (original platoon)
                fprintf('Q%.0d tries to merge with Q%.0d \n', quadrotors{i}.ID, quadrotors{i}.platoon.FP.vehicle{end}.ID)
                quadrotors{i}.u = quadrotors{i}.mergeWithQuadrotor(quadrotors{i}.platoon.FP.vehicle{end}, highway, v);
            else
                % Follower
                quadrotors{i}.u = quadrotors{i}.followPlatoon(highway);
            end
        
            
        elseif numUnsafeTargets == 1
            
            % One target is unsafe, apply safe control w.r.t. to that
            % target.
            % If quadrotor is a Leader, it stays a Leader 
            % If quadrotor is a Follower, it becomes an EmergLeader and
            % splits platoon
            if strcmp(quadrotors{i}.q, 'Follower')
                fprintf('Q %.0d splits platoon %.0d \n', quadrotors{i}.ID, quadrotors{i}.platoon.ID)
                quadrotors{i}.splitPlatoon();
            end
            % from this point, quadrotors{i} is EmergLeader
            if ~quadrotors{i}.safeI
                quadrotors{i}.u = uSafeI;
            elseif ~quadrotors{i}.safeFQ
                quadrotors{i}.u = uSafeFQ;
            elseif ~quadrotors{i}.safeBQ
                quadrotors{i}.u = uSafeBQ;
            end
            
            
        elseif sum(valC<0) == 0
            % It's unsafe because of exceeding speed limit, apply any safe
            % control to slow down
            quadrotors{i}.u = uSafeI;
            
        else
            % Number of unsafe targets more than 1. Change height
            quadrotors{i}.abandonPlatoon;
            abandonedQ = [abandonedQ, i];
            fprintf('Q%.0d descends \n', quadrotors{i}.ID)
        end
            
    end
    
    Intruder.u = Intruder.followPath(tSteps, intruderPath, Intruder.vMax);
    
    
    
    % ------------- Plot simulation ------------- %

    
    figure(f1)
    % Unplot abandoned quadrotors
    for i = 1:length(abandonedQ)
        quadrotors{abandonedQ(i)}.unplotPosition();
        quadrotors{abandonedQ(i)}.unplotSafeV();
    end
    
    % Remove abandoned quadrotors from list of quadrotors
    quadrotors(abandonedQ) = [];
    numQ = numQ-length(abandonedQ);
    fprintf('Number of quadrotors remaining: %.0d \n', numQ)

    
    
    % Plot remaining quadrotors
    for i = 1:3
        quadrotors{i}.plotPosition();
    end
    Intruder.plotPosition('red');    

    % Plot reach set around Intruder, FQ and BQ for Q2
    quadrotors{2}.plotSafeV_2(Intruder, safeV, hsafeVI);hsafeVI.LineColor = 'r';
    quadrotors{2}.plotSafeV_2(quadrotors{1}, safeV, hsafeVFQ); hsafeVFQ.LineColor = 'k';
    quadrotors{2}.plotSafeV_2(quadrotors{3}, safeV, hsafeVBQ);hsafeVBQ.LineColor = [0 0.5 0];
    
    axis([10,50,10,50])
    title(sprintf('t = %.1f',t(k)));
    drawnow;
    savefig(sprintf('fig/Intruder2_%.0d.fig',p)); 
    print(sprintf('fig/Intruder2_%.0d.pdf',p), '-dpdf'); 
    p = p+1;
    
    
    % Plot snapshots
    if ~isempty(tplot) && t(k) >= tplot(1)
        plotnum = plotnum+1;
        figure(f2)
        s = subplot(spR,spC,plotnum);
        copyobj(f1.Children.Children, s);
        tplot(1) = [];
        title(sprintf('t = %.1f',t(k))); axis equal; axis([10, 50, 10, 50]);%box on;
    end
    drawnow;
    
    % Update states for all quadrotors
    Intruder.updateState(Intruder.u);
    for i = 1:length(quadrotors)
        quadrotors{i}.updateState(quadrotors{i}.u);
    end

    
end


end



