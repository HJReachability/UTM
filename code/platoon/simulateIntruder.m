function simulateIntruder(from_checkpoint, save_graphics, output_directory, visualize_vehicle_on)
% function simulateIntruder(from_checkpoint, save_graphics, output_directory)
%
% Simulates Simulates a platoon behavior with intruder.
%
% Inputs:  from_checkpoint  - whether to load previous checkpoint
%          save_graphics    - whether to export graphics (for making an
%                            animation)
%          output_directory - for graphics export
%                             - requires export_fig package
%                             - don't include a "/" at the end
%
% Author: Qie Hu, 2015-07-14

% Default input parameters
if nargin<1
  from_checkpoint = false;
end

if nargin<2
  save_graphics = false;
end

if nargin<3
  output_directory = 'saved_graphics';
end

if nargin<4
  visualize_vehicle_on = false;
end

cvx_quiet true % Shuts cvx up if using MPC controller in followPath
warning('off','all')

% Checkpoint directory and file
check_point_dir = 'checkpoints';
check_point = [check_point_dir '/' mfilename '.mat'];

%% Initialization
if from_checkpoint
  disp('Loading from last checkpoint')
  load(check_point)
  
else
  % Make directory if needed
  system(['mkdir ' check_point_dir]);
  
  tEnd = 10;                 % End of simulation time
  dt = 0.1;                 % Sampling time
  t = 0:dt:tEnd;            % Time horizon
  tSteps = 5;               % MPC horizon in followPath
  gridSize = 50;
  v = 3;                  % Target highway speed
  d = 2;                  % Safe separation distance
  
  % Get reachability information
  [reachInfo, safeV] = generateReachInfo();
  
  
  % ===== Create highway here =====
  z1 = [0 0];
  z2 = [gridSize gridSize];
  hw = highway(z1, z2, v);
  
  % ===== Create intruder here =====
  qiX0 = [25;17];       % Initial position
  qiXt = [-5;17];       % Final position
  qi = quadrotor(6, [qiX0(1);-3;qiX0(2);0], reachInfo); % ID = 6
  qiPath = highway(qiX0, qiXt, v);
  
  
  % ===== Create a platoon of quadrotors on the highway =====
  leaderPos = [18;18];
  leaderVel = [2;2];
  Nqr = 4;
  p = popPlatoon(hw, leaderPos, leaderVel, Nqr, 1);
  
  qrs = cell({});
  for j = 1:Nqr
    qrs{j} = p.vehicles{j};
  end
  
  % Set up video writer
  writerObj = VideoWriter(sprintf('Intruder1_5.mp4'),'MPEG-4');    % Create object for writing video
  writerObj.FrameRate = 10;                   % Number of frames / sec
  open(writerObj);                            % Start videoWriter
  
  % Set up plotting
  f1 = figure;    % Normal version
  
  % Figure 1
  figure(f1)
  hw.hwPlot; hold on
  
  colors = lines(Nqr);
  for j = 1:Nqr
    qrs{j}.plotPosition(colors(j,:,:));
  end
  qi.plotPosition('red');                   % Intruder in red
  
  xlabel('x');    ylabel('y');
  title(sprintf('t = %.01d',t(1)));
  axis([0, gridSize, 0, gridSize]); %axis equal;
  
  % Figure 2. Visualize initial vehicle properties
  if visualize_vehicle_on
    f2 = figure;
    figure(f2)
    visualizeVehicles(qrs);
    title(['t=' num2str(t(1))])
    drawnow;
  end
  
  % Save graphics if needed
  if save_graphics
    system(['mkdir ' output_directory])
    % Export large figure as fig and pdf
    savefig(f1,sprintf('%s/%s_1.fig',output_directory,mfilename));
    print(f1,sprintf('%s/%s_1.pdf',output_directory,mfilename), '-dpdf');
    %         export_fig([output_directory '/' mfilename '1'], '-png') % Export figure as png
    
    % Figure 3. Set up variables for plotting snap shots
    f3 = figure;
    tplot = [0.1 1.1 2.0 6.0];
    numPlots = length(tplot);
    spC = ceil(sqrt(numPlots));
    spR = ceil(numPlots/spC);
    plotnum = 0;
  end
  
%   % Record videos
%   ax = f1.CurrentAxes;
%   ax.Units = 'pixels';
%   pos = ax.Position;
%   ti = ax.TightInset;
%   rect = [-ti(1), -ti(2), pos(3)+ti(1)+ti(3), pos(4)+ti(2)+ti(4)+10];
%   frame = getframe(ax,rect);
%   writeVideo(writerObj,frame);
  
  % Starting index in the simulation loop (needed for saving checkpoints)
  kStart = 2;
  
  qRem = 1:Nqr;   % Index of vehicles remaining in this altitude
  qAb = [];       % Index of vehicles that abandoned this altitude
  
end % end if from_checkpoint


for k = kStart:length(t)
  fprintf('t = %.01f \n', t(k))
  
  for j = qRem % for each vehicle remaining
    
    fprintf('Q%.0d is %s, in P%.0d \n', qrs{j}.ID, qrs{j}.q, qrs{j}.p.ID)
    
    % ------------- Check Safety ------------- %
    
    % Check safety w.r.t. Intruder
    [qrs{j}.safeI, uSafeI] = qrs{j}.isSafe(qi, safeV);
    qrs{j}.safeIhist = cat(2, qrs{j}.safeIhist, qrs{j}.safeI);
    
    
    % Check safety w.r.t. FQ
    if strcmp(qrs{j}.q, 'Leader')
      
      % If it's a Leader, check w.r.t to trailing
      % vehicle of front platoon
      if qrs{j}.p.FP == qrs{j}.p % If no platoon in front, set to safe
        qrs{j}.safeFQ = true;
      else
        [qrs{j}.safeFQ, uSafeFQ] = ...
          qrs{j}.isSafe(qrs{j}.p.FP.vehicles{qrs{j}.p.FP.n}, safeV);
      end
      
    else
      
      % Follower
      [qrs{j}.safeFQ, uSafeFQ] = qrs{j}.isSafe(qrs{j}.FQ, safeV);
      
    end
    qrs{j}.safeFQhist = cat(2, qrs{j}.safeFQhist, qrs{j}.safeFQ);
    
    
    % Check safety w.r.t. BQ
    if qrs{j}.idx == find(qrs{j}.p.slotStatus==1,1,'last')
      %             % Trailing quadrotor, check w.r.t. leader of back platoon
      %             BP = qrs{j}.platoon.BP;
      %             if BP == qrs{j}.platoon % If no platoon at back, set to safe
      qrs{j}.safeBQ = true;
      %             else
      %                 [qrs{j}.safeBQ, uSafeBQ] = qrs{j}.isSafe(BP.vehicles{1}, safeV);
      %             end
    else
      [qrs{j}.safeBQ, uSafeBQ] = qrs{j}.isSafe(qrs{j}.BQ, safeV);
    end
    qrs{j}.safeBQhist = cat(2, qrs{j}.safeBQhist, qrs{j}.safeBQ);
    
    % Total number of unsafe targets
    numUnsafeTargets = ~qrs{j}.safeI + ~qrs{j}.safeFQ + ~qrs{j}.safeBQ;
    
    
    % ------------- Compute Control ------------- %
    
    if numUnsafeTargets == 0
      %  Safe w.r.t. Intruder, FQ & BQ, can do anything
      
      if strcmp(qrs{j}.q, 'Leader')
        
        if qrs{j}.p.FP ~= qrs{j}.p && ...
            qrs{j}.p.FP.loIdx + qrs{j}.p.loIdx <= qrs{j}.p.FP.nmax
          % There is another platoon in front and the total number
          % of vehicles between this platoon and the one in front
          % is less than max number of vehicles. Join platoon in front.
          fprintf('Q%.0d merging with P%.0d \n', qrs{j}.ID, qrs{j}.p.FP.ID)
          qrs{j}.u = qrs{j}.mergeWithPlatoon(qrs{j}.p.FP);
          
        else
          % Follow path
          qrs{j}.u = qrs{j}.followPath(tSteps, hw, qrs{j}.vMax);
        end
        
      else
        % Follower
        qrs{j}.u = qrs{j}.followPlatoon();
      end % if strcmp(qrs{j}.q, 'Leader')
      
      
    elseif numUnsafeTargets == 1
      
      % One target is unsafe, apply safe control w.r.t. to that
      % target.
      % If quadrotor is a Leader, it stays a Leader
      % If quadrotor is a Follower, it becomes an EmergLeader and
      % splits platoon
      if strcmp(qrs{j}.q, 'Follower')
        fprintf('Q%.0d splits P%.0d \n', qrs{j}.ID, qrs{j}.p.ID)
        qrs{j}.splitPlatoon();
      end
      % from this point, qrs{j} is EmergLeader
      if ~qrs{j}.safeI
        qrs{j}.u = uSafeI;
      elseif ~qrs{j}.safeFQ
        qrs{j}.u = uSafeFQ;
      elseif ~qrs{j}.safeBQ
        qrs{j}.u = uSafeBQ;
      end
      
    else
      % Number of potential collisions more than 1. Change height.
      qrs{j}.p.removeVehicle(qrs{j});
      qAb = [qAb, qrs{j}];
      fprintf('Q%.0d descends \n', qrs{j}.ID)
    end % if numUnsafeTargets == 0
    
  end %for j = qRem
  
  qi.u = qi.followPath(tSteps, qiPath, qi.vMax);
  
  
  % --------- Check safe separation distance w.r.t. intruder --------- %
  
  for j = qRem
    if qrs{j}.sepDist(qi) < d
      % Within safe separation distance. Change height.
      fprintf('Collision between Q%.0d and Intruder. \n', qrs{j}.ID);
      qrs{j}.p.removeVehicle(qrs{j});
      % Remove abandoned quadrotors from list of quadrotors
      qRem(qRem == j) = [];
      qAb = [qAb, j];
    end
  end
  
  
  
  % ------------------ Plot simulation ------------------ %
  
  % ------- Normal View --------- %
  figure(f1)
  % Unplot abandoned quadrotors
  while ~isempty(qAb)
    qrs{qAb(1)}.unplotPosition();
    for ii = 1:(length(qrs{qAb(1)}.hsafeV)-1)
      if ~isempty(qrs{qAb(1)}.hsafeV{ii})
        qrs{qAb(1)}.unplotSafeV(qrs{ii});
      end
    end
    if ~isempty(qrs{qAb(1)}.hsafeV{end})
      qrs{qAb(1)}.unplotSafeV(qi);
    end
    qAb(1) = [];
  end
  fprintf('Number of quadrotors remaining: %.0d \n', length(qRem))
  
  % Update remaining vehicles' state, plot vehicle positions and position history
  qi.updateState(qi.u);
  qi.plotPosition('red');
  for j = qRem
    qrs{j}.updateState(qrs{j}.u);
    qrs{j}.plotPosition(colors(j,:,:));
  end
  title(sprintf('t = %.01f',t(k)));
  drawnow;
  %     savefig(sprintf('fig/Intruder1_%.0d.fig',k));
  %     print(sprintf('fig/Intruder1_%.0d.pdf',k), '-dpdf');
  
  % ----- Visualize vehicle properties ----- %
  if visualize_vehicle_on
    figure(f2)
    clf(f2)
    visualizeVehicles(qrs);
    title(sprintf('t = %.01f',t(k)));
    drawnow;
  end
  
%   % ----- Record video ----- %
%   frame = getframe(ax,rect);
%   writeVideo(writerObj,frame);
%   
  
  % ---- Save graphics if specified ---- %
  if save_graphics
    % Export large figure as fig and pdf
    savefig(f1,sprintf('%s/%s_%.0d.fig',output_directory,mfilename,k));
    print(f1,sprintf('%s/%s_%.0d.pdf',output_directory,mfilename,k), '-dpdf');
    % export_fig([output_directory '/' mfilename num2str(i)], '-png')
    
    % Create figure with subplots
    if ~isempty(tplot) && t(k) >= tplot(1)
      plotnum = plotnum+1;
      figure(f3)
      s = subplot(spR,spC,plotnum);
      copyobj(f1.Children.Children, s);
      tplot(1) = [];
      title(sprintf('t = %.01f',t(k)));
      axis equal; axis([0, gridSize, 0, gridSize]);%box on;
      if plotnum == 3, xlabel('x'); ylabel('y'); end
    end % end if ~isempty(tplot) && t(i) >= tplot(1)
    drawnow;
    
    
  end % end if save_graphics
  
  %     disp('Saving checkpoint...')
  %     save(check_point)
  
end % end main simulation loop

close(writerObj);

end % end function