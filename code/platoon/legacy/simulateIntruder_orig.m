% Simulates a platoon behavior with intruder

clear; close all;


cvx_quiet true
warning('off','all')

tEnd = 7;                                % End of simulation time
dt = 0.1;                                   % Sampling time
t = 0:dt:tEnd;                              % Time horizon
tSteps = 5;                                 % MPC horizon in followPath

gridSize = 70;
v = 3.5;                                    % Target highway speed

% Get reachability information
[reachInfo, safeV] = generateReachInfo();


% Highway
z1 = [0 0];
z2 = [gridSize gridSize];
hw = highway(z1, z2, v);

% intruding quadrotor
qiX0 = [40;30];       % Initial position
qiXt = [10;20];       % Final position
qi = quadrotor(0, dt, [qiX0(1);-4;qiX0(2);-2], reachInfo);
qiPath = highway(qiX0, qiXt, v);


% create a platoon of quadrotors on the highway
leaderPos = [23;23];
leaderVel = [4;4];
Nqr = 4;
p = popPlatoon(hw, leaderPos, leaderVel, Nqr, 1);

qr = [];
for j = 1:Nqr
    qr = [qr,p.vehicle(j)];
end

u = zeros(2,Nqr);


% % Set up video writer
% writerObj = VideoWriter(sprintf('Intruder1.mp4'),'MPEG-4');    % Create object for writing video
% writerObj.FrameRate = 10;                   % Number of frames / sec
% open(writerObj);                            % Start videoWriter

% Set up plotting
f1 = figure;                                    % Normal version
f2 = figure;                                    % Snapshots

% Figure 1
figure(f1)
hw.hwPlot; hold on

colors = lines(Nqr);
for j = 1:Nqr
    qr(j).plotPosition(colors(j,:,:));
end
qi.plotPosition('red');                   % Intruder in red

xlabel('x');    ylabel('y');

ds = hw.ds;
vx = v*ds(1);
vy = v*ds(2);

title(sprintf('t = %.01d',t(1)));
axis([0, gridSize, 0, gridSize]); %axis equal;

% Set up variables for plotting snap shots
tplot = [0 3.3 6.2];
spC = 2;
spR = 2;
plotnum = 0;


% % Record videos
% ax = f1.CurrentAxes;
% ax.Units = 'pixels';
% pos = ax.Position;
% ti = ax.TightInset;
% rect = [-ti(1), -ti(2), pos(3)+ti(1)+ti(3), pos(4)+ti(2)+ti(4)+10];
% frame = getframe(ax,rect);
% writeVideo(writerObj,frame);


for k = 2:length(t)
    
    fprintf('t = %.01f \n', t(k))
    qAb = [];
    
    for j = 1:Nqr % for each quadrotor

        valCx = zeros(1,3);      % Safety value due to collision
        
        fprintf('Q%.0d is %s, in P%.0d \n', qr(j).ID, qr(j).q, qr(j).p.ID)
        
        % ------------- Check Safety ------------- %
        
        % Check safety w.r.t. Intruder
        [qr(j).safeI, uSafeI, ~, valSx, valCx(1)] = qr(j).isSafe(qi, safeV);
        qr(j).safeIhist = cat(2, qr(j).safeIhist, qr(j).safeI);
        
        
        % Check safety w.r.t. FQ
        if strcmp(qr(j).q, 'Leader') || strcmp(qr(j).q, 'EmergLeader')
            % If it's a Leader or EmergLeader, check w.r.t to trailing
            % vehicle of front platoon
            if qr(j).p.FP == qr(j).p % If no platoon in front, set to safe
                if valSx >= 0,  qr(j).safeFQ = true;
                else            qr(j).safeFQ = false;
                end
            else
                [qr(j).safeFQ, uSafeFQ, ~, ~, valCx(2)] = ...
                    qr(j).isSafe(qr(j).p.FP.vehicle(qr(j).p.FP.n), safeV);
            end
        else % Follower
            [qr(j).safeFQ, uSafeFQ, ~, ~, valCx(2)] = qr(j).isSafe(qr(j).FQ, safeV);
        end
        qr(j).safeFQhist = cat(2, qr(j).safeFQhist, qr(j).safeFQ);

        
        % Check safety w.r.t. BQ
        if qr(j).idx == qr(j).p.n 
%             % Trailing quadrotor, check w.r.t. leader of back platoon
%             BP = qr(j).platoon.BP;
%             if BP == qr(j).platoon % If no platoon at back, set to safe
                    if valSx >= 0,  qr(j).safeBQ = true;
                    else            qr(j).safeBQ = false;
                    end
%             else
%                 [qr(j).safeBQ, uSafeBQ] = qr(j).isSafe(BP.vehicle(1), safeV);
%             end
        else
            [qr(j).safeBQ, uSafeBQ, ~, ~, valCx(3)] = qr(j).isSafe(qr(j).BQ, safeV);
        end
        qr(j).safeBQhist = cat(2, qr(j).safeBQhist, qr(j).safeBQ);

        % Total number of unsafe targets
        numUnsafeTargets = ~qr(j).safeI + ~qr(j).safeFQ + ~qr(j).safeBQ;
        
        
        % ------------- Compute Control ------------- %
        
        if numUnsafeTargets == 0     
            %  Safe w.r.t. Intruder, FQ & BQ, can do anything
            if strcmp(qr(j).q, 'Leader')
                qr(j).u = qr(j).followPath(tSteps, hw, qr(j).vMax);
            elseif strcmp(qr(j).q, 'EmergLeader') 
                % Join platoon in front (original platoon)
                fprintf('Q%.0d merging with P%.0d \n', qr(j).ID, qr(j).p.FP.ID)
                qr(j).u = qr(j).mergeWithPlatoon(qr(j).p.FP);
            else
                % Follower
                qr(j).u = qr(j).followPlatoon();
            end
        
            
        elseif numUnsafeTargets == 1
            
            % One target is unsafe, apply safe control w.r.t. to that
            % target.
            % If quadrotor is a Leader, it stays a Leader 
            % If quadrotor is a Follower, it becomes an EmergLeader and
            % splits platoon
            if strcmp(qr(j).q, 'Follower')
                fprintf('Q%.0d splits P%.0d \n', qr(j).ID, qr(j).p.ID)
                qr(j).splitPlatoon();
            end
            % from this point, qr(j) is EmergLeader
            if ~qr(j).safeI
                qr(j).u = uSafeI;
            elseif ~qr(j).safeFQ
                qr(j).u = uSafeFQ;
            elseif ~qr(j).safeBQ
                qr(j).u = uSafeBQ;
            end
                        
        elseif valSx<0 && sum(valCx<0) == 0
            % It's unsafe because it exceeded speed limit, but no potential
            % collision, apply any safe control to slow down
            qr(j).u = uSafeI;
           
        elseif valSx<0 && sum(valCx<0) == 1
            % It's unsafe because it exceeded speed limit and one potential
            % collision, if uSafe to avoid collision also slows down
            % vehicle, then it's OK
            cMode = find(valCx<0);
            switch cMode
                case 1, uSafe = uSafeI;
                case 2, uSafe = uSafeFQ;
                case 3, uSafe = uSafeBQ;
            end
            if qr(j).x(qr(j).vdim)+uSafe < qr(j).vMax
                qr(j).u = uSafe;
            else
                qr(j).p.removeVehicle(qr(j));
                qAb = [qAb, j];
                fprintf('Q%.0d descends \n', qr(j).ID)
            end
            
        else
            % Number of potential collisions more than 1. Change height.
            keyboard
            qr(j).p.removeVehicle(qr(j));
            qAb = [qAb, j];
            fprintf('Q%.0d descends \n', qr(j).ID)
        end
            
    end %for j = 1:Nqr
    
    qi.u = qi.followPath(tSteps, qiPath, qi.vMax);
    
    
    
    % ------------------ Plot simulation ------------------ %

    % ------- Normal --------- %
    figure(f1)
    % Unplot abandoned quadrotors
    for i = 1:length(qAb)
        qr(qAb(i)).unplotPosition();
    end
    
    % Remove abandoned quadrotors from list of quadrotors
    qr(qAb) = [];
    Nqr = Nqr-length(qAb);
    fprintf('Number of quadrotors remaining: %.0d \n', Nqr)

    
    % Plot remaining quadrotors
    for j = 1:Nqr
        qr(j).plotPosition(colors(j,:,:));
    end
    qi.plotPosition('red');
    title(sprintf('t = %.01f',t(k)));
    drawnow;
%     savefig(sprintf('fig/Intruder1_%.0d.fig',k)); 
%     print(sprintf('fig/Intruder1_%.0d.pdf',k), '-dpdf'); 
    
       
    % ----- Snapshots ----- %
    if ~isempty(tplot) && t(k) >= tplot(1)
        plotnum = plotnum+1;
        figure(f2)
        s = subplot(spR,spC,plotnum);
        copyobj(f1.Children.Children, s);
        tplot(1) = [];
        title(sprintf('t = %.01f',t(k))); 
        axis equal; axis([0, gridSize, 0, gridSize]);%box on;
        if plotnum == 3, xlabel('x'); ylabel('y'); end
    end
    
    drawnow;
    
%     % ----- Record video ----- %
%     frame = getframe(ax,rect);
%     writeVideo(writerObj,frame);
    
    % ----- Update states for all quadrotors ----- %
    qi.updateState(qi.u);
    for j = 1:Nqr
        qr(j).updateState(qr(j).u);
    end

    
end

% close(writerObj);





