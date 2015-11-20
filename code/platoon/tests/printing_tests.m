function printing_tests()
addpath('..')
tfm = TFM;
hw1 = Highway([0 0], [100 0]);
hw2 = Highway([-100 0], [-100 100]);

q1 = Quadrotor([0 0 0 0]);
q2 = Quadrotor([10 0 0 0]);
q3 = Quadrotor([-100 0 50 0]);

tfm.regVehicle(q1);
tfm.regVehicle(q2);
tfm.regVehicle(q3);
tfm.addHighway(hw1);
tfm.addHighway(hw2);

p1 = Platoon(q1, hw1, tfm);
p2 = Platoon(q3, hw2, tfm);
q1.printInfo;
q1.printPreOrder;

p1.printInfo;

p1.insertVehicle(q2, 2);
p1.printInfo;

disp('=============')

p1.printPreOrder;

disp('=============')
hw1.printPreOrder;

disp('=============')
tfm.printPreOrder;
end