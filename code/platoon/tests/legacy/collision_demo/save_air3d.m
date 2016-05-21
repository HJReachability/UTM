
% from demo: (r, v_a, v_b, max_a, max_b, tMax) = (5, 5, 5, 1, 1, 5)
RADIUS = 5;
V_A = 5;
V_B = 5;
MAX_A = 1;
MAX_B = 1;

% compute g
[ data, g, data0 ] = air3D('low', RADIUS, V_A, V_B, MAX_A, MAX_B, 5);

save datafile.mat data
save gfile.mat g