clear all; close all

cvx_quiet true

tEnd = 2;                  % End of simulation time
dt = 0.1;                   % Sampling time
t = 0:dt:tEnd;              % Time horizon
tsteps = 5;                 % time steps to look ahead in MPC
v = 3;

% Get reachability information
[reachInfo, safeV] = generateReachInfo();

% put some quadrotors in a horizontal line and !!create a platoon!!
qr1 = quadrotor(1, dt, [0; 0; -20; 0], reachInfo);
qr2 = quadrotor(2, dt, [-25; 0; 35; 0], reachInfo);
qr2.idx = 2;
qr = [qr1; qr2];

Nqr = length(qr);
u = zeros(2,Nqr);

% Highway
z0 = [-30 -15];
z1 = [80 40];
hw = highway(z0, z1, v);

xt = 6;
target = [xt 0.5*xt];


% Visualize initial set
f1 = figure;
f2 = figure;

figure(f1)
% subplot(1,2,1)
hw.hwPlot; hold on
ht = plot(target(1), target(2), 'ro');

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

ds = hw.ds;
vx = v*ds(1)/norm(ds);
vy = v*ds(2)/norm(ds);

title(['t=' num2str(t(1))])
% axis equal
xlim([-25 40])
ylim([-25 40])
% return
drawnow;

% export_fig('E:\Normal2\1', '-png')
tplot = [1.5 2.8 7 12];
numPlots = length(tplot);
spC = ceil(sqrt(numPlots));
spR = ceil(numPlots/spC);
plotnum = 0;

for i = 2:length(t)
%     [qr(1).x(2) qr(1).x(4) sqrt(qr(1).x(2)^2 + qr(1).x(4)^2)]
    for j = 1:Nqr % Each quadrotor
        
        % Check safety
        
        if j>1, [safe, uSafe] = qr(j).isSafe(qr(j-1), safeV);
        else     safe         = 1;
        end
        
        if safe
%             disp([num2str(j) 'is safe'])
            
            if ~isempty([qr.platoon]) % If there's a platoon
%                 keyboard
                disp('Platoon!')
                p = [qr.platoon];
                pj = p.ID;
                
                if j == pj
                    disp('Leading')
                    u(:,j) = qr(j).followPath(tsteps, hw, v);
                else
                    if isempty(qr(j).platoon)
                        disp('Merging into platoon')
                        u(:,j) = qr(j).mergeWithQuadrotor( ...
                            qr(pj).platoon.vehicle{qr(pj).platoon.n}, ...
                            hw, v);
                    else
                        disp('Following platoon')
                        u(:,j) = qr(j).followPlatoon(hw);
                    end
                end
            else
                disp('No platoon.')
                u(:,j) = qr(j).mergeOnHighway(hw, target, v);
            end
        else
            disp([num2str(j) 'is unsafe!'])
            u(:,j) = uSafe;
        end
    end
    
    figure(f1)
    for j = 1:Nqr
        qr(j).updateState(u(:,j));
        qr(j).plotPosition();
        
%         qr(j).plotVelocity();
        
        if j>1

            if ~isempty(qr(1).platoon)
                delete(qr(j-1).hsafeV{j});
                delete(ht);
                qr(j).plotSafeV(qr(j-1), safeV);
                qr(j).plotMergePlatoonV(qr(1), hw);
                
                xPh = qr(1).platoon.phantomPosition(hw, qr(j).ID);
                if ~exist('hph', 'var')
                    hph = plot(xPh(1), xPh(2), 'bo');
                else
                    hph.XData = xPh(1);
                    hph.YData = xPh(2);
                end
            else
                qr(j-1).plotSafeV(qr(j), safeV, colors{j});

            end
        else
            qr(j).plotMergeHighwayV(hw, target);
        end
    end
    
%     subplot(1,2,1)
%     axis equal
    xlim([-25 40])
    ylim([-25 40])

    title(['t=' num2str(t(i))])
    drawnow;
%     export_fig(['E:\Normal2\' num2str(i)], '-png')

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
