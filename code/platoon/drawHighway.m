function drawHighway(fig,highwayPath)

figure(fig);
N = 500;
s = linspace(0,1,N);
rpath = highwayPath(s);
h = plot(rpath(1,:),rpath(2,:),'-.');
h.Color = [0.5, 0.5, 0.5];
xlabel('x'); ylabel('y'); axis equal;

end