function K= createLQR(qr)

%creates an lqr controller for the quadrotor to follow the path
% qr is the quadrotor object, and qr.A and qr.B describe the dynamics of
% the linear dynamics of the quadrotor


if nargin<1
    A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
    B = [0 0; 1 0; 0 0; 0 1];
else
    A=qr.A;
    B=qr.B;
end


K=lqr(A,B,diag([1 2 1 2]),eye(2)*.01);
end
