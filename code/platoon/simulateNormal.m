clear all; close all

cvx_quiet true

tEnd = 20;                  % End of simulation time
dt = 0.1;                   % Sampling time
t = 0:dt:tEnd;              % Time horizon
tsteps = 5;                 % time steps to look ahead in MPC
v = 3;

% Get reachability information
[reachInfo, safeV] = generateReachInfo();

% put some quadrotors in a horizontal line and !!create a platoon!!
qr1 = quadrotor(1, dt, [0; 0; 0; 0], reachInfo);
qr2 = quadrotor(2, dt, [-8; 0; 3; 0], reachInfo);
qr3 = quadrotor(3, dt, [-3; 0; -13; 0], reachInfo);
qr4 = quadrotor(4, dt, [-8; 0; -28; 0], reachInfo);
qr5 = quadrotor(5, dt, [-30; 0; 10; 0], reachInfo);
qr = [qr1; qr2; qr3; qr4; qr5];
% qr = [qr1; qr2; qr3];


Nqr = length(qr);
u = zeros(2,Nqr);

% Highway
z1 = [-30 -15];
z2 = [80 40];
hw = highway(z1, z2, v);

% Put first vehicle in platoon
p = platoon(qr1, hw);

% Visualize initial set
f1 = figure;
f2 = figure;

figure(f1)
% subplot(1,2,1)
hw.hwPlot; hold on

colors = lines(Nqr);
for j = 1:Nqr
    %     subplot(1,2,1)
    qr(j).plotPosition(colors(j,:,:));
    
end

xlabel('x');    ylabel('y');

ds = hw.ds;
vx = v*ds(1);
vy = v*ds(2);

title(['t=' num2str(t(1))])
axis equal

drawnow;
tplot = [1 4 7 10];
numPlots = length(tplot);
spC = ceil(sqrt(numPlots));
spR = ceil(numPlots/spC);
plotnum = 0;
for i = 2:length(t)
    for j = 1:Nqr % Each quadrotor
        
        % Check safety
        % For simplicity (for now), assume qr1 is always safe, and
        % subsequent qr(j)s check safety only with qr(j-1)
        if j>1, [safe, uSafe] = qr(j).isSafe(qr(j-1), safeV);
        else     safe         = 1;
        end
        
        if safe
            
            if strcmp(qr(j).q, 'Leader')
                disp('Leading')
                u(:,j) = qr(j).followPath(tsteps, hw);
                
            elseif strcmp(qr(j).q, 'Follower')
                disp('Following platoon')
                u(:,j) = qr(j).followPlatoon;
                
            elseif strcmp(qr(j).q, 'Free')
                disp('Merging into platoon')
                u(:,j) = qr(j).mergeWithPlatoon(hw.ps);
                
            else
                error('Invalid mode!')
                
            end
            
        else
            disp([num2str(j) 'is unsafe!'])
            u(:,j) = uSafe;
            
        end
    end
    
    figure(f1)
    for j = 1:Nqr
        qr(j).updateState(u(:,j));
        qr(j).plotPosition(colors(j,:,:));
        
        %         qr(j).plotVelocity();
        
        %         if j>1
        %             qrs(j).plotSafeV(qrs(j-1), safeV);
        %         end
    end
    
    %     if ~isempty(qr(1).platoon)
    % %         qr(3).plotSafeV(qr(2), safeV);
    %         qr(3).plotMergePlatoonV(qr(1), highway);
    %     end
    
    
    %     subplot(1,2,1)
    title(['t=' num2str(t(i))])
    
    drawnow;
    %     export_fig(['E:\New Folder\normal1_' num2str(i)], '-png','-transparent')
    
    %     if ~isempty(tplot) && t(i) >= tplot(1)
    %         plotnum = plotnum+1;
    %         figure(f2)
    %         s = subplot(spR,spC,plotnum);
    %         copyobj(f1.Children.Children, s);
    %         tplot(1) = [];
    %
    %         title(['t=' num2str(t(i))])
    %
    %         axis equal
    %         xlim([-30 40]); ylim([-30 40])
    %         if plotnum == 3; xlabel('x'); ylabel('y'); end
    %     end
    %
    %     drawnow;
end
