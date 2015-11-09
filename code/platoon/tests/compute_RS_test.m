function compute_RS_test()
addpath('../RS_core')

target = [0 10 0 0];

% Big, coarse set
visualize = 0;
[grids, datas, tau] = quad_abs_target_2D(target,  visualize);
gridLim = ...
  [grids{1}.min-1 grids{1}.max+1; grids{2}.min-1 grids{2}.max+1];
[~, TD_out, TTR_out] = recon2x2D(tau, grids, datas, gridLim, tau(end));

% [TD_out, TTR_out] = quad_abs_target_4D;

% tfm.computeRS('qr_abs_target_V')

N = 12;
spC = ceil(sqrt(N));
spR = ceil(N/spC);

hp = figure;
hv = figure;
for i = 1:N
  % Project 4D reachable set to position space
  v = [-6+i 10];
  figure(hp)
  [g2D, data2D] = proj2D(TTR_out.g, [0 1 0 1], ...
    TTR_out.g.N([1 3]), TTR_out.value, v);
  
  subplot(spR, spC, i)
  contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:13)
  title(['v = ' num2str(v)])
  
  % Project 4D reachable set to position space
  p = -5 + 10 * rand(1,2);
  figure(hv)
  [g2D, data2D] = proj2D(TTR_out.g, [1 0 1 0], ...
    TTR_out.g.N([2 4]), TTR_out.value, p);
  
  subplot(spR, spC, i)
  contour(g2D.xs{1}, g2D.xs{2}, data2D, 0:13)
  title(['p = ' num2str(p)])

  
end