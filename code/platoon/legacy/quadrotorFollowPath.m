function quadrotorFollowPath()
clear all; close all

cvx_quiet true
ID = 1;
dt = 0.1;
Tmax = 5;

% vmax = 1;
qr = quadrotor(ID, dt, [0; 0; 0; 0]);

% udes = 0.5;
pdim = [1 3];
vdim = [2 4];

t = 0:dt:Tmax;

hp = cell(2,1);
hv = cell(2,1);
hu = cell(2,1);
hr = cell(2,1);
tcolors = {'k', [0 0.4 0]};
ucolors = {'k', [0 0.4 0]};
f = figure;
nrow = 3;
ncol = 1;

rstyles = {'none','none'};

tsteps = 10;
for i = 2:length(t)
    % Compute control and update state
    disp(t(i))
    u = qr.followPath(tsteps, @rpath, 3);
    qr.updateState(u(:,1));
    
    % Plot position vs. time
    subplot(nrow,ncol,1)
    for j = 1:length(pdim)
        if i == 2
            hp{j} = plot(t(1:i), qr.xhist(pdim(j),:), '-', 'marker', rstyles{j}, ...
                'color', tcolors{j}); hold on
            if j == length(pdim)
                legend([hr{1} hr{2} hp{1} hp{2}], ...
                    {'r_x','r_y','p_x','p_y'})
            end
        else
            hp{j}.XData = t(1:i);
            hp{j}.YData = qr.xhist(pdim(j),:);
        end
    end
    ylabel('Position')
    
    % Plot velocity vs. time
    subplot(nrow,ncol,2)
    for j = 1:length(vdim)
        if i == 2
            hv{j} = plot(t(1:i), qr.xhist(vdim(j),:), '-', 'marker', rstyles{j}, ...
                'color', tcolors{j}); hold on
            
            if j == length(vdim)
                legend([hv{1} hv{2}], {'v_x','v_y'})
                xlim([0 1.2*Tmax])
                ylabel('Velocity')                
            end
        else
            hv{j}.XData = t(1:i);
            hv{j}.YData = qr.xhist(vdim(j),:);
        end
    end

    
    % Plot control vs. time
    subplot(nrow,ncol,3)
    for j = 1:2
        if i == 2
            hu{j} = plot(t(2:i), qr.uhist(j,:), '.-', 'color', ucolors{j}); hold on
            
            if j == 2
                legend([hu{1} hu{2}], {'u_x','u_y'})
                xlim([0 1.2*Tmax])
                xlabel('Time (s)')
                ylabel('Control')
            end
        else
            hu{j}.XData = t(2:i);
            hu{j}.YData = qr.uhist(j,:);
        end
    end

    drawnow;
end

figure;
plot(qr.xhist(1,:), qr.xhist(3,:), 'b.-'); hold on

Ns = 100;
s = linspace(0,1,Ns);
p = rpath(s);
plot(p(1,:),p(2,:), 'r-'); hold on
% xlim([-0.5 0.5])
% keyboard
end

function p = rpath(s)
% function p = rpath(s)
% 
% parameterization of a path to be followed; 0 <= s <= 1

% p = [0.2*ones(1,length(s)); 5*s];
p = [1 + 10*s; 5*s];

end