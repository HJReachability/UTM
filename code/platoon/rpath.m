function p = rpath(s,x0,xt)
%
% computes p = sequence of points along a straight path from x0 to xt

p = x0*ones(1,length(s))+(xt-x0)*s;
end
