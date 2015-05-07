clear all; close all

cvx_quiet true
ID = 1;
dt = 0.1;
Tmax = 15;
T = 2;

v = vehicle(ID, dt);

udes = 0.5;
pdim = [1 5];
vdim = [2 6];

t = 0:dt:Tmax;
% traj_t = 0.5:0.1:T;
% traj = [0.1*traj_t.^2; -0.05*traj_t.^3];
traj_t = [0 5 10 15];
traj = [0 5 5 10; 0 0 5 0];

hp = cell(2,1);
hv = cell(2,1);
hu = cell(2,1);
hr = cell(2,1);
tcolors = {'k', [0 0.4 0]};
ucolors = {'k', [0 0.4 0]};
f = figure;
nrow = 3;
ncol = 1;

% Plot reference trajectory
rcolors = {'r','c'};
rstyles = {'.','+'};
for j = 1:length(pdim)
    subplot(nrow,ncol,1)
    hr{j} = plot(traj_t, traj(j,:), '-', 'marker', rstyles{j}, ...
        'color', rcolors{j}); hold on
end
xlim([0 1.2*Tmax])

for i = 2:length(t)
    % Compute control and update state
    disp(t(i))
    u = v.trackTraj(traj, traj_t, t(i), T);
    v.updateState(u(:,1));
    
    % Plot position vs. time
    subplot(nrow,ncol,1)
    for j = 1:length(pdim)
        if i == 2
            hp{j} = plot(t(1:i), v.xhist(pdim(j),:), '-', 'marker', rstyles{j}, ...
                'color', tcolors{j}); hold on
            if j == length(pdim)
                legend([hr{1} hr{2} hp{1} hp{2}], ...
                    {'r_x','r_y','p_x','p_y'})
            end
        else
            hp{j}.XData = t(1:i);
            hp{j}.YData = v.xhist(pdim(j),:);
        end
    end
    ylabel('Position')
    
    % Plot velocity vs. time
    subplot(nrow,ncol,2)
    for j = 1:length(vdim)
        if i == 2
            hv{j} = plot(t(1:i), v.xhist(vdim(j),:), '-', 'marker', rstyles{j}, ...
                'color', tcolors{j}); hold on
            
            if j == length(vdim)
                legend([hv{1} hv{2}], {'v_x','v_y'})
                xlim([0 1.2*Tmax])
                ylabel('Velocity')                
            end
        else
            hv{j}.XData = t(1:i);
            hv{j}.YData = v.xhist(vdim(j),:);
        end
    end

    
    % Plot control vs. time
    subplot(nrow,ncol,3)
    for j = 1:2
        if i == 2
            hu{j} = plot(t(2:i), v.uhist(j,:), '.-', 'color', ucolors{j}); hold on
            
            if j == 2
                legend([hu{1} hu{2}], {'u_x','u_y'})
                xlim([0 1.2*Tmax])
                xlabel('Time (s)')
                ylabel('Control')
            end
        else
            hu{j}.XData = t(2:i);
            hu{j}.YData = v.uhist(j,:);
        end
    end

    drawnow;
%     keyboard
end

figure;
plot(v.xhist(1,:),v.xhist(5,:), 'b.-'); hold on
plot(traj(1,:),traj(2,:), 'r.-'); hold on