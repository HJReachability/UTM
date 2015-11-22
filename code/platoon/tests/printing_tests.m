function printing_tests()
% printing_tests()
% Tests functions that print information of nodes

addpath('..')
tfm = TFM;
hw1 = Highway([0 0], [100 0]);
hw2 = Highway([-100 0], [-100 100]);

q1 = Quadrotor([0 0 0 0]);
q2 = Quadrotor([10 0 0 0]);
q3 = Quadrotor([-100 0 50 0]);
pl1 = Plane([50 50 0 0]);

tfm.regVehicle(q1);
tfm.regVehicle(q2);
tfm.regVehicle(q3);
tfm.regVehicle(pl1);
tfm.addHighway(hw1);
tfm.addHighway(hw2);

p1 = Platoon(q1, hw1, tfm);
p2 = Platoon(q3, hw2, tfm);

p1.insertVehicle(q2, 2);
p2.insertVehicle(pl1, 3);

disp('===== p1.printPreOrder =====')
p1.printPreOrder;

disp('===== hw1.printPreOrder =====')
hw1.printPreOrder;

disp('====== tfm.printPreOrder =====')
tfm.printPreOrder;

disp('====== p1.printBreadthFirst =====')
p1.printBreadthFirst;

disp('====== hw1.printBreadthFirst =====')
hw1.printBreadthFirst;

disp('====== tfm.printBreadthFirst =====')
tfm.printBreadthFirst;
end