function quadrotorFollowQuadrotor()
dt = 0.1;
qr1 = quadrotor(1, dt, [2; 0; 2; 0], @rpath);
qr2 = quadrotor(1, dt, [1; 0; -2; 0], @rpath);
qr3 = quadrotor(1, dt, [-2; 0; 0; 0], @rpath);

figure;
h1t = plot(qr1.xhist(1,:), qr1.xhist(3,:), 'b-'); hold on
h2t = plot(qr2.xhist(1,:), qr2.xhist(3,:), 'k-');
h3t = plot(qr3.xhist(1,:), qr3.xhist(3,:), 'c-');
h1p = plot(qr1.x(1), qr1.x(3), 'b.');
h2p = plot(qr2.x(1), qr2.x(3), 'k.');
h3p = plot(qr3.x(1), qr3.x(3), 'c.');


Ns = 100;
s = linspace(0,1,Ns);
p = rpath(s);
plot(p(1,:),p(2,:), 'r-')

legend('qr1','qr2','rpath')
drawnow;

tMax = 5;
t = 0:dt:tMax;

tsteps = 10;

for i = 2:length(t)
    u1 = qr1.followPath(tsteps);
    u2 = qr2.followQuadrotor(qr1, tsteps);
    u3 = qr3.followQuadrotor(qr2, tsteps);
    
    qr1.updateState(u1(:,1));
    qr2.updateState(u2(:,1));
    qr3.updateState(u3(:,1));
    
    h1t.XData = qr1.xhist(1,:);
    h1t.YData = qr1.xhist(3,:);
    h1p.XData = qr1.x(1);
    h1p.YData = qr1.x(3);
    
    h2t.XData = qr2.xhist(1,:);
    h2t.YData = qr2.xhist(3,:);
    h2p.XData = qr2.x(1);
    h2p.YData = qr2.x(3);
    
    h3t.XData = qr3.xhist(1,:);
    h3t.YData = qr3.xhist(3,:);
    h3p.XData = qr3.x(1);
    h3p.YData = qr3.x(3);
    
    drawnow;
%     keyboard
end
end

function p = rpath(s)
% function p = rpath(s)
% 
% parameterization of a path to be followed; 0 <= s <= 1

% p = [0.2*ones(1,length(s)); 5*s];
p = [1 + 10*s; 5*s];

end