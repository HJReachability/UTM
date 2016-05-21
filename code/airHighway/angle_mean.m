function th_out = angle_mean(th_in, thresh)
% Computes the average angle taking into account wrapping at +/-pi
%
% thresh - threshold for being considered close to pi

if nargin<2
  thresh = 5 * pi / 180;
end

if (min(th_in)-(-pi) < thresh) && (pi-max(th_in) < thresh)
  th_in(th_in<0) = th_in(th_in<0) + 2*pi;
end

th_out = mean(th_in);
end