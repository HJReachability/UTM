clear all; close all
[ dataC, gC, tauC ] = quad3Dcollision;
[ dataS, gS, tauS ] = quad3DspeedLimit;
tau = tauC;
g = gC;
save('quad3D')

load('quad3D')

x = [2.1 0 5 0 0 0];
t = 3.5;
[~, ~, g6D, valueC] = recon2x3D(tau, g, dataC, g, dataC, x, t);

ind = min(length(tau), find(tau<=t,1,'last')+1);
valueSx = eval_u(g, dataS(:,:,:,ind), [g6D.xs{1}(:) g6D.xs{2}(:) g6D.xs{3}(:)]);
valueSy = eval_u(g, dataS(:,:,:,ind), [g6D.xs{4}(:) g6D.xs{5}(:) g6D.xs{6}(:)]);
valueS = min(valueSx, valueSy);
valueS = reshape(valueS, g6D.shape);

gradC = extractCostates(g6D, valueC);
gradCx = calculateCostate(g6D, gradC, x)

gradS = extractCostates(g6D, valueS);
gradSx = calculateCostate(g6D, gradS, x)

value = min(valueC, valueS);

valuex = eval_u(g6D, value, x)

grad = extractCostates(g6D, value);
gradx = calculateCostate(g6D, grad, x)


valuesd = signedDistanceIterative(g6D, value, 'veryHigh');
gradsd = extractCostates(g6D, valuesd);
gradx = calculateCostate(g6D, gradsd, x)