[datax, datay, g1, g2, tau]=quad2D_joinHighwayPlatoon([0 0 0 0],0);

liveV.datax=datax;
liveV.datay=datay;
liveV.g1=g1;
liveV.g2=g2;
liveV.tau=tau;
liveV.gdim=2;

save quad_liveness_2x2D liveV

clear liveV

[~,~,liveV.g,liveV.data,~,~,liveV.grad,~]=recon2x2D(tau,g1,datax,g2,datay,...
[g1.min-1 g1.max+1;g2.min-1 g2.max+1]);                                       
liveV.tau=tau;
liveV.gdim=4;

save quad_liveness_4D liveV
