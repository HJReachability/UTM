[dataC,g,tau]=quad2Dcollision(2,0);

save quad_safe_2x2D dataC g tau

clear safeV

[~,~,g,dataC,~,~,grad,~]=recon2x2D(tau,g,dataC,g,dataC,...
[g.min-1 g.max+1;g.min-1 g.max+1]);                                       

save quad_safe_4D g dataC grad tau
