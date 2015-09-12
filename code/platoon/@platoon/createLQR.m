function K= createLQR(obj)
%creates an lqr controller for the quadrotor to follow the path
% obj is the quadrotor object, and obj.A and obj.B describe the dynamics of
% the linear dynamics of the quadrotor

K = lqr(obj.A, obj.B, diag([1 2 1 2]), eye(2)*.01);
end
