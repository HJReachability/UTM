function simulateNormal2(from_checkpoint, save_graphics, output_directory, visualize_vehicle_on)
% function simulateNormal2(from_checkpoint, save_graphics,
%                                                         output_directory)
%
% Simulates two vehicles merging onto the highway. The first vehicle forms
% a platoon on the highway, and the second vehicle joins the platoon.
%
% Inputs:  from_checkpoint  - whether to load previous checkpoint
%          save_graphics    - whether to export graphics (for making an
%                            animation)
%          output_directory - for graphics export
%                             - requires export_fig package
%                             - don't include a "/"
%          visualize_vehicle_on -debugging tool
%
% Description:
% There are just two vehicles in this simulation. The first vehicle merges
% onto the highway by first moving in a straight line towards its target,
% and then switching to the reachability-based liveness controller once it
% enters the target set. The second vehicle also tries to join the highway
% at the same target, but the first vehicle finishes merging first. So the
% second vehicle instead joins the new platoon created by the first
% vehicle. Both vehicles switch to the safety controller if they enter the
% safety reachable set with respect to the other vehicle.
%
% Mo Chen, 2015-10-02

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
  visualize_vehicle_on = 0;
end

cvx_quiet true % Shuts cvx up if using MPC controller in followPath

% Checkpoint directory and file
check_point_dir = 'checkpoints';
check_point = [check_point_dir '/' mfilename '.mat'];

%% Initialization
if from_checkpoint
  disp('Loading from last checkpoint')
  load(check_point)
  
else
  tEnd = 20;                  % End of simulation time
  dt = 0.1;                   % Sampling time (this has to be 0.1!)
  t = 0:dt:tEnd;              % Time horizon
  tsteps = 5;                 % time steps to look ahead in MPC
  v = 3;
  
  f1 = figure;
  f2 = figure;
  
  % Get reachability information
  [reachInfo, safeV] = generateReachInfo();
  
  % Highway
  z0 = [-30 -15];
  z1 = [80 40];
  hw = highway(z0, z1, v);
  
%   % put some quadrotors in a horizontal line and !!create a platoon!!
  qr1 = quadrotor(1, [0; 0; -20; 0], reachInfo);
  qr2 = quadrotor(2, [-25; 0; 35; 0], reachInfo);
%   qr2 = quadrotor(2, [-30; 0; -15; 0], reachInfo);
  qrs = {qr1; qr2};
%   p = popPlatoon(hw, 0.2, hw.speed, 2, 1);
%   qrs = p.vehicles(1:p.n);
%   qrs{2}.x(qrs{2}.pdim) = p.phantomPosition(5);
  
  Nqr = length(qrs);
  u = zeros(2,Nqr);
  colors = lines(Nqr);
  
  % Target for entering highway
  xt = 6;
  target = [xt 0.5*xt];
  
  % Visualize initial set
  figure(f1)
  hw.hwPlot; hold on
  ht = plot(target(1), target(2), 'color', colors(1,:,:));
  
  for j = 1:Nqr
    qrs{j}.plotPosition(colors(j,:,:));
  end
  
  xlabel('x');
  ylabel('y');
  title(['t=' num2str(t(1))])
  
  xlim([-25 40])
  ylim([-25 40])
  
  % Visualize initial vehicle properties
  if visualize_vehicle_on
    figure(f2)
    visualizeVehicles(qrs);
    title(['t=' num2str(t(1))])
    drawnow;
  end

  
  % Save graphics if needed
  if save_graphics
    system(['mkdir ' output_directory])
    export_fig([output_directory '/' mfilename '1'], '-png') % Export figure as png    
    
    % Create figure with subplots
    f3 = figure;
    tplot = [1.5 2.8 7 12];
    numPlots = length(tplot);
    spC = ceil(sqrt(numPlots));
    spR = ceil(numPlots/spC);
    plotnum = 0;
  end
  
  % Starting index in the simulation loop (needed for saving checkpoints)
  iStart = 2;
end

%% Main simulation loop
for i = iStart:length(t)
  for j = 1:Nqr % Each quadrotor
    % Check safety
    if j>1
      [safe, uSafe] = qrs{j}.isSafe(qrs{j-1}, safeV);
    else
      safe = 1;
    end

    if safe
      if ~isempty(hw.ps) % If there's a platoon
        if strcmp(qrs{j}.q, 'Leader')
          disp('Leading')
          u(:,j) = qrs{j}.followPath(tsteps, hw, v);
          
        else % if strcmp(qr(j).q, 'Leader')
          if strcmp(qrs{j}.q, 'Free')
            disp('Merging into platoon')
            u(:,j) = qrs{j}.mergeWithPlatoon(hw.ps);
            
          elseif strcmp(qrs{j}.q, 'Follower') % if isempty(qr(j).platoon)
            disp('Following platoon')
            u(:,j) = qrs{j}.followPlatoon;
            
          else
            error('Unknown mode!')
            
          end % if strcmp(qr(j).q, 'Free')
        end % if strcmp(qr(j).q, 'Leader')
        
      else % if ~isempty(hw.ps)
        disp('No platoon.')
        u(:,j) = qrs{j}.mergeOnHighway(hw, target);
       
      end % if ~isempty(hw.ps)
      
    else % if safe
      disp([num2str(j) 'is unsafe!'])
      u(:,j) = uSafe;
    end % if safe
  end % for j = 1:Nqr
  
  figure(f1)
  for j = 1:Nqr
    qrs{j}.updateState(u(:,j));
    qrs{j}.plotPosition(colors(j,:,:));
  end
  
  if ~isempty(hw.ps) % If there's a platoon
%     delete(qrs{1}.hsafeV{2});
%     delete(ht);
%     delete(qrs{1}.hmergeHighwayV);
    
%     qrs{2}.plotSafeV(qrs{1}, safeV);
    
    % qr(2).p already assigned inside qr(2).mergeWithPlatoon(hw.ps) on
    % line 92. This issue is that qr(2) is now a follower so there's
    % nothing to plot, and qr(2).pJoin is now empty
%     qrs{2}.plotMergePlatoonV;
    
    
    xPh = qrs{1}.p.phantomPosition(qrs{1}.p.n + 1);
    
    if ~exist('hph', 'var')
      hph = plot(xPh(1), xPh(2), 'color', colors(2,:,:));
    else
      hph.XData = xPh(1);
      hph.YData = xPh(2);
    end % ~exist('hph', 'var')
    
  else % ~isempty(hw.ps)
%     qrs{1}.plotMergeHighwayV(target);
%     qrs{1}.plotSafeV(qrs{2}, safeV);
    
  end % ~isempty(hw.ps)
  
  figure(f1)
  xlim([-25 40])
  ylim([-25 40])
  
  title(['t=' num2str(t(i))])
  drawnow;
  
  % Save graphics if specified
  if save_graphics
    % Export large figure as png
    export_fig([output_directory '/' mfilename num2str(i)], '-png')
    
    % Create figure with subplots
    if ~isempty(tplot) && t(i) >= tplot(1)
      plotnum = plotnum+1;
      figure(f3)
      s = subplot(spR,spC,plotnum);
      copyobj(f1.Children.Children, s);
      tplot(1) = [];
      
      title(['t=' num2str(t(i))])
      
      axis equal
      xlim([-30 40]); ylim([-30 40])
      if plotnum == 3; xlabel('x'); ylabel('y'); end
    end
  end
  
  % Visualize vehicle properties

  
  if visualize_vehicle_on
    figure(f2)
    clf(f2)
    visualizeVehicles(qrs);
    title(['t=' num2str(t(i))])
    drawnow;
  
  end
  

  % Save checkpoint
  iStart = i+1;
  disp('Saving checkpoint...')
%   save(check_point)
end % for i = 2:length(t)
keyboard
end % end of function