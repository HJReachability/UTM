function angle_diff_test()

disp(['True angle difference = 0.2; computed angle difference = ' ...
  num2str(angle_diff(0.1, -0.1))])

disp(['True angle difference = 0.2; computed angle difference = ' ...
  num2str(angle_diff(pi-0.1, -pi+0.1))])

end