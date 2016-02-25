function min_sij_approach()
numVehicles = 3;

% 1 has lower safety value
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;
Cs = {C};

% Three agents in conflict
% Case 12, 23
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(2,3) = 2;
C(3,2) = 1;
Cs = [Cs; C];

% Case 12, 32
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(3,2) = 2;
C(2,3) = 1;
Cs = [Cs; C];


% Case 12, 13
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(1,3) = 2;
C(3,1) = 1;
Cs = [Cs; C];

% Case 12, 23, 31
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(2,3) = 2;
C(3,2) = 1;

C(3,1) = 2;
C(1,3) = 1;
Cs = [Cs; C];

% Case 12, 23, 13
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(2,3) = 1;
C(3,2) = 2;

C(1,3) = 2;
C(3,1) = 1;
Cs = [Cs; C];

%% Test all cases
checkCases(Cs);
end