function abs_angle_diff_test()
% function abs_angle_diff_test()
%
% Tests the abs_angle_diff function

addpath('..')

disp(['True angle difference = 0.2; computed angle difference = ' ...
  num2str(abs_angle_diff(0.1, -0.1))])

disp(['True angle difference = 0.2; computed angle difference = ' ...
  num2str(abs_angle_diff(pi-0.1, -pi+0.1))])

end