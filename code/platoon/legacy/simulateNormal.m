function simulateNormal(from_checkpoint, save_graphics, output_directory,visualize_vehicle_on)
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
% Description:
% The first vehicle (vehicle 1) is already on the highway, and is the 
% leader of a platoon consisting of itself. The other four vehicles join 
% the% platooning, targetting the first available position behind the 
% leader. Each vehicle i checks safety with vehicle i-1, while following
% the liveness controller to merge with the platoon. The leader simply
% follows the highway.
%
% Mo Chen, 2015-10-02

addpath('RS_core')

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
  % Make directory if needed
  system(['mkdir ' check_point_dir]);
  
  tEnd = 50;                  % End of simulation time
  dt = 0.1;                   % Sampling time
  t = 0:dt:tEnd;              % Time horizon
  tsteps = 5;                 % time steps to look ahead in MPC
  v = 3;
  
  % Get reachability information
  [reachInfo, safeV] = generateReachInfo();
  
  % ===== Specify quadrotors here =====
  qr1 = quadrotor(1, [0; 0; 0; 0], reachInfo);
  qr2 = quadrotor(2, [-8; 0; 3; 0], reachInfo);
  qr3 = quadrotor(3, [-3; 0; -13; 0], reachInfo);
  qr4 = quadrotor(4, [-8; 0; -28; 0], reachInfo);
  qr5 = quadrotor(5, [-30; 0; 10; 0], reachInfo);
  qrs = {qr1; qr2; qr3; qr4; qr5};
  Nqr = length(qrs);
  u = zeros(2,Nqr);
  
  % ===== Create highway here =====
  z1 = [-30 -15];
  z2 = [160 80];
  
  hw = highway(z1, z2, v, true);
    
  % ===== Create platoon here =====
  % Put first vehicle in platoon
  p = platoon(qr1, hw);
  
  % Visualize initial setup
  f1 = figure;

  if visualize_vehicle_on
    f2 = figure;
  end
  
  figure(f1)
  % Plot highway
  hw.hwPlot; hold on
  
  % Plot quadrotor positions
  colors = lines(Nqr);
  for j = 1:Nqr
    qrs{j}.plotPosition(colors(j,:,:));
  end
  xlabel('x');
  ylabel('y');
  title(['t=' num2str(t(1))])
  axis equal
  drawnow;
  
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
    tplot = [1 4 7 10];
    numPlots = length(tplot);
    spC = ceil(sqrt(numPlots));
    spR = ceil(numPlots/spC);
    plotnum = 0;
  end
  
  % Starting index in the simulation loop (needed for saving checkpoints)
  iStart = 2;
  
end

for i = iStart:length(t)
  for j = 1:Nqr % Each quadrotor
    % Check safety
    % For simplicity (for now), assume qr1 is always safe, and
    % subsequent qr(j)s check safety only with qr(j-1)
    if j>1
      [safe, uSafe] = qrs{j}.isSafe(qrs{j-1}, safeV);
    else
      safe = 1;
    end
    
    % Compute control
    if safe
      % Control if safe depends on mode
      if strcmp(qrs{j}.q, 'Leader')
        disp('Leading')
        u(:,j) = qrs{j}.followPath(tsteps, hw);
        
      elseif strcmp(qrs{j}.q, 'Follower')
        disp('Following platoon')
        u(:,j) = qrs{j}.followPlatoon;
        
      elseif strcmp(qrs{j}.q, 'Free')
        disp('Merging into platoon')
        u(:,j) = qrs{j}.mergeWithPlatoon(hw.ps);
        
      else
        error('Invalid mode!')
        
      end
      
    else
      % Safety control if unsafe
      disp([num2str(j) 'is unsafe!'])
      u(:,j) = uSafe;
      
    end
  end
  
  % Update vehicle state, plot vehicle positions and position history
  figure(f1)
  for j = 1:Nqr
    qrs{j}.updateState(u(:,j));
    qrs{j}.plotPosition(colors(j,:,:));
  end
  title(['t=' num2str(t(i))])
  
  %   % Visualize vehicle properties
  %   figure(f2)
  %   clf(f2)
  %   visualizeVehicles(qrs);
  %   title(['t=' num2str(t(i))])
  drawnow;
  
  % Save graphics if specified
  if save_graphics
    % Export large figure as png
    export_fig([output_directory '/' mfilename num2str(i)], f1, '-png')
    
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
      if plotnum == 3
        xlabel('x');
        ylabel('y');
      end
    end % end if ~isempty(tplot) && t(i) >= tplot(1)
  end % end if save_graphics
  
  iStart = i+1;
  
  %disp('Saving checkpoint...')
  %save(check_point)
end % end main simulation loop
end % end function