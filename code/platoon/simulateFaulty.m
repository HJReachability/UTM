clear all; close all

cvx_quiet true

tEnd = 40;                  % End of simulation time
dt = 0.1;                   % Sampling time
t = 0:dt:tEnd;              % Time horizon
tsteps = 5;                % time steps to look ahead in MPC
v = 3;

% Get reachability information
[reachInfo, safeV] = generateReachInfo();

% Load QR positions (already on highway)
var = load('simulateNormalWorkspace.mat');

qr1 = quadrotor(1, dt, [0; 0; 0; 0], reachInfo);
qr2 = quadrotor(2, dt, [-8; 0; 3; 0], reachInfo);
qr3 = quadrotor(3, dt, [-3; 0; -13; 0], reachInfo);
qr4 = quadrotor(4, dt, [-8; 0; -28; 0], reachInfo);
qr5 = quadrotor(5, dt, [-30; 0; 10; 0], reachInfo);
qr_init = [qr1; qr2; qr3; qr4; qr5];

Nqr = length(qr_init);
u = zeros(2,Nqr);

qr = var.qr;

for i=1:Nqr
   qr(i).hsafeV = qr_init(i).hsafeV;
   qr(i).xhist = qr(i).x;
end

clear var
close all
% return
% Highway
x0 = -60;    x1 = 140;
y0 = 0.5*x0; y1 = 0.5*x1;
highway = @(s) [(1-s)*x0 + s*x1; (1-s)*y0 + s*y1];
target = [4 2];
highwayd = highway([0 1]);

% RHighway
Rx0 = 140;    Rx1 = -60;
Ry0 = 0.5*x1; Ry1 = 0.5*x0;
Rhighway = @(s) [(1-s)*Rx0 + s*Rx1; (1-s)*Ry0 + s*Ry1];
target = [4 2];
Rhighwayd = Rhighway([0 1]);

% Visualize initial set
f1 = figure;
f2 = figure;

figure(f1)
% subplot(1,2,1)
hhw = plot(highwayd(1,:), highwayd(2,:), 'k--'); hold on

colors = {'r', 'b', 'k', [0 0.5 0], [1 0 1]};
for j = 1:Nqr
%     subplot(1,2,1)
    qr(j).plotPosition(colors{j});
%     
%     subplot(1,2,2)
%     qr(j).plotVelocity(colors{j});
end

% subplot(1,2,1)
% qr(2).plotSafeV(qr(1), safeV);
% xlim([-10 25]); ylim([-10 10]);
xlabel('x');    ylabel('y');

% ds = highway(1) - highway(0);
% vx = v*ds(1)/norm(ds);
% vy = v*ds(2)/norm(ds);

% legend([hhw qr{4}.hpxpy qr{4}.hsafeV], {'Highway','Position and velocity','Keep-out set'})
% ho = plot( qr(1).x(qr(1).pdim(1)) - 3*(2*sqrt(2))*ds(1)/norm(ds), ...
%     qr(1).x(qr(1).pdim(2)) - 3*(2*sqrt(2))*ds(2)/norm(ds), 'o');
title(['t=' num2str(t(1))])
axis equal
% xlim([-30 40]); ylim([-30 40])

drawnow;
tplot = [0 1 2.1 4.5];
numPlots = length(tplot);
spC = ceil(sqrt(numPlots));
spR = ceil(numPlots/spC);
plotnum = 0;
fault = 0;
normal = 1;

for i = 2:length(t)
    for j = 1:Nqr % Each quadrotor
        if (j==3 && t(i) > 0)
            %Remove fautly QR from platoon
            if(normal == 1)
                qr(j).removeFromPlatoon();
                normal = 0;
            end
%             disp('crazy quad!')
%             calculate worst control relative to follower
            u(:,j) = qr(j).followPath(tsteps, Rhighway, v);
            fault = 3;
%             u(:,j) = qr(j).faultyControl(qr(j+1), safeV);%faultyControl(qr(j), qr(j+1), safeV);
        else
            % Check safety
            if j>1, [safe, uSafe] = qr(j).isSafe(qr(j-1), safeV);
%             elseif(j==4 && t(i) > 7)
%                 safe = 1;
            else safe = 1;
            end
            
            % Check safety from faulty QR
            if(j>1 && fault ~= 0)
                [safe, uSafe] = qr(j).isSafe(qr(fault), safeV);
            end

            if safe
%                 disp([num2str(j) 'is safe'])

%                 if ~isempty([qr.platoon]) % If there's a platoon
                    p = [qr.platoon];
                    pj = p.ID;

                    if j == pj
%                         disp('Leading')
                        u(:,j) = qr(j).followPath(tsteps, highway, v);
                    else
                        if isempty(qr(j).platoon)
%                             disp('Merging into platoon')
                            u(:,j) = qr(j).mergeWithQuadrotor( ...
                                qr(pj).platoon.vehicle{qr(pj).platoon.n}, ...
                                highway, v);
                        else
%                             disp('Following platoon')
                            if(strcmp(qr(j).q, 'EmergLeader'))
                                u(:,j) = qr(j).mergeWithQuadrotor( ...
                                qr(pj), ...
                                highway, v);
                            else
                                u(:,j) = qr(j).followPlatoon(highway);
                            end
                        end
                    end
%                 else
%                     disp('No platoon.')
%                     u(:,j) = qr(j).mergeOnHighway(highway, target, v);
%                 end
            else
                disp([num2str(j) 'is unsafe!'])
                qr(j).q = 'EmergLeader'; 
                u(:,j) = uSafe;
            end
        end
    end
    
    figure(f1)
    for j = 1:Nqr

            qr(j).updateState(u(:,j));
            qr(j).plotPosition();
            
            
%         qr(j).plotVelocity();

% Plot reachable sets
        if (j == 5)
            qr(j).plotSafeV(qr(j-2), safeV);
            qr(j).plotSafeV(qr(j-1), safeV);
        end
        if (j == 4)
            qr(j).plotSafeV(qr(j-1), safeV);
        end        
    end

    axis equal
    axis manual
    xlim([-20 100]); ylim([-20 50]);
    
% Save all plots
%     fig = gcf;
%     FileName=['plots/plot_',num2str(i)];
%     print(fig,FileName,'-dpng');
    
    title(['t=' num2str(t(i))]);
    export_fig(['C:\Users\mochen72\Desktop\simulateFaulty\' num2str(i)], '-png','-transparent')
%     ho.XData = qr(1).x(qr(1).pdim(1)) - 3*(2*sqrt(2))*ds(1)/norm(ds);%*vx * tFollow;
%     ho.YData = qr(1).x(qr(1).pdim(2)) - 3*(2*sqrt(2))*ds(2)/norm(ds);%*vy * tFollow;
    
    drawnow;
     
    if ~isempty(tplot) && t(i) >= tplot(1)
        plotnum = plotnum+1;
        figure(f2)
        s = subplot(spR,spC,plotnum);
        copyobj(f1.Children.Children, s);
        tplot(1) = [];
        
        title(['t=' num2str(t(i))])
        
        axis equal
        axis manual
        xlim([-20 100]); ylim([-20 50]);
        if plotnum == 3; xlabel('x'); ylabel('y'); end
    end
    
    drawnow;
end
