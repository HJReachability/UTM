function [ht, hp] = drawQuadrotor(fig,ht,hp,quadrotor)
% LEGACY! Use quadrotor.plotSafeV instead
%

error('LEGACY! Use quadrotor.plotSafeV instead')

pdim = quadrotor.pdim;
figure(fig);hold on;

if isempty(ht)
    ht = plot(quadrotor.xhist(pdim(1),:), quadrotor.xhist(pdim(2),:)); hold on 
    hp = scatter(quadrotor.x(pdim(1)), quadrotor.x(pdim(2))); 
    hp.CData = ht.Color; hp.MarkerFaceColor = ht.Color; hp.LineWidth = 2.5;
else
    ht.XData = quadrotor.xhist(pdim(1),:); %Path
    ht.YData = quadrotor.xhist(pdim(2),:);
    hp.XData = quadrotor.x(pdim(1)); % Present position
    hp.YData = quadrotor.x(pdim(2));
end

end