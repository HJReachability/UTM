function simulateHighwayMerge(debug, save_graphics, output_directory)
% function simulateNormal(from_checkpoint, save_graphics, output_directory)
%
% Simulates 4 quadrotors joining a platoon that initially has a single
% quadrotor. This file is a good template for new simulations!
%
% Inputs:  from_checkpoint  - whether to load previous checkpoint
%          save_graphics    - whether to export graphics (for making an
%                            animation)
%          output_directory - for graphics export
%                             - requires export_fig package
%                             - don't include a "/" at the end
%
% Mo Chen, 2015-07-08

% Default input parameters
if nargin<1
  debug = false;
end

if nargin<2
  save_graphics = true;
end

if nargin<3
  output_directory = 'saved_graphics';
end

cvx_quiet true % Shuts cvx up if using MPC controller in followPath

% Checkpoint directory and file
check_point_dir = 'checkpoints';
check_point = [check_point_dir '/' mfilename '.mat'];

%% Initialization
if debug && exist(check_point,'file')
  disp('Loading from last checkpoint')
  load(check_point)
  
else
  % Make directory if needed
  system(['mkdir ' check_point_dir]);
  
  tEnd = 30;                  % End of simulation time
  dt = 0.1;                   % Sampling time
  t = 0:dt:tEnd;              % Time horizon
  
  % Create the highways
  hw1 = highway([-50 0], [150 0]);
  hw2 = highway([-50 -25], [150 75]);
  
  hws = {hw1; hw2};
  
  popuHwyConnections(hws);
  
  % Initialize figures
  f0 = figure;
  figure(f0)
  hwColors = lines(2);
  for i = 1:length(hws)
    hws{i}.hwPlot(hwColors(i,:,:)); hold on
  end
  
  % Get reachability information
  [~, safeV] = generateReachInfo();
  
  % ===== Specify quadrotors and platoons here =====
  % First platoon on hw1
  ID1 = 1;
  numVehicles = 3;
  p1 = popPlatoon(hw1, 0.275, hw1.speed, numVehicles, ID1);
  
  % Second platoon on hw2
  ID1 = 4;
  numVehicles = 4;
  p2 = popPlatoon(hw2, 0.15, hw1.speed, numVehicles, ID1);
  
  % Specify which of the 4 quadrotors in p2 will join p1
  IDjoin = [4 6];
 
  % All vehicles
  qrs = [p1.vehicles(1:p1.n); p2.vehicles(1:p2.n)];
  Nqr = length(qrs);
  colors = lines(Nqr);
  u = zeros(2,Nqr);
  tsteps = 5;
  
  % Specify the list of vehicles with whom safety should be checked
  for i = 1:Nqr
    % Check safety with FQ and BQ
    qrs{i}.sList = {qrs{i}.FQ; qrs{i}.BQ};
    
    % If vehicle is in p2, check safety with all vehicles in p1 (for now)
    if i>p1.n
      qrs{i}.sList = [qrs{i}.sList; p1.vehicles(1:p1.n)];
    end
  end  

  % Plot initial positions
  figure(f0);
  for i = 1:Nqr
    qrs{i}.plotPosition;
  end
  
  xlabel('x');
  ylabel('y');
  title(['t=' num2str(t(1))])
  xlim([-30 60])
  axis equal
  drawnow;
  
  if debug
    f1 = figure;
    f2 = figure;
    
    % Visualize vehicle properties
    figure(f1)
    p1.pVisualize;
    title(['t=' num2str(t(1))])
    
    figure(f2)
    p2.pVisualize;
    title(['t=' num2str(t(1))])
  end
  
  % Save graphics if needed
  if save_graphics
    system(['mkdir ' output_directory])
    export_fig([output_directory '/' mfilename '1'], f0, '-png') % Export figure as png
    
    % Create figure with subplots
    f4 = figure;
    tplot = [1 4 7 14];
    numPlots = length(tplot);
    spC = ceil(sqrt(numPlots));
    spR = ceil(numPlots/spC);
    plotnum = 0;
  end
  
  drawnow;
  
  % Starting index in the simulation loop (needed for saving checkpoints)
  iStart = 2;
end

for i = iStart:length(t)
  for j = 1:Nqr % Each quadrotor in platoon 1
    % Check safety
    [safe, uSafe, valuex] = qrs{j}.checkSafety(safeV);
    
    % Compute control
    if all(safe)
      % If all safety indicators are true, then use liveness controller
%       [~, dist] = hw1.highwayPos(qrs{j}.x(qrs{j}.pdim));
      if any(qrs{j}.ID == IDjoin) && qrs{j}.x(qrs{j}.pdim(2)) > -15
        IDjoin(IDjoin == qrs{j}.ID) = [];
        qrs{j}.sList = p1.vehicles;
        qrs{j}.abandonPlatoon;
      end
      
      % Control if safe depends on mode
      if strcmp(qrs{j}.q, 'Leader')
        disp([num2str(qrs{j}.ID) ' Leading'])
        u(:,j) = qrs{j}.followPath(tsteps, qrs{j}.p.hw);
        
      elseif strcmp(qrs{j}.q, 'Follower')
        disp([num2str(qrs{j}.ID) ' Following platoon'])
        u(:,j) = qrs{j}.followPlatoon;
        
      elseif strcmp(qrs{j}.q, 'Free')
        disp([num2str(qrs{j}.ID) ' Free'])
        u(:,j) = qrs{j}.mergeWithPlatoon(p1);
        
      else
        error('Invalid mode!')
        
      end
      
    else
      % If some safety indicator is false, then use safety controller
      % corresponding to lowest safety value
      % Safe w.r.t. FQ, so unsafe w.r.t. BQ
      disp([num2str(qrs{j}.ID) 'is unsafe!'])
      [~, iUnsafe] = min(valuex);
      u(:,j) = uSafe{iUnsafe};
      
    end
  end

  % Update vehicle states, plot vehicle positions and position history
  figure(f0)
  for j = 1:Nqr
    qrs{j}.updateState(u(:,j));
    qrs{j}.plotPosition(colors(j,:));
  end
  title(['t=' num2str(t(i))])
  
    if debug
    % Visualize vehicle properties
    figure(f1)
    clf(f1)
    p1.pVisualize;
    title(['t=' num2str(t(1))])

    figure(f2)
    clf(f2)
    p2.pVisualize;
    title(['t=' num2str(t(1))])
  end
  
  % Save graphics if specified
  if save_graphics
    % Export large figure as png
    export_fig([output_directory '/' mfilename num2str(i)], f0, '-png')
    
    % Create figure with subplots
    if ~isempty(tplot) && t(i) >= tplot(1)
      plotnum = plotnum+1;
      figure(f4)
      s = subplot(spR,spC,plotnum);
      copyobj(f0.Children.Children, s);
      tplot(1) = [];
      
      title(['t=' num2str(t(i))])
      
      axis equal
      xlim([-30 40]); ylim([-30 40])
      if plotnum == 3
        xlabel('x');
        ylabel('y');
      end
    end % end if ~isempty(tplot) && t(i) >= tplot(1)
  end % end if save_graphics
  
  drawnow;
  iStart = i+1;

%   disp('Saving checkpoint...')
%   save(check_point)
end % end main simulation loop
keyboard
end % end function