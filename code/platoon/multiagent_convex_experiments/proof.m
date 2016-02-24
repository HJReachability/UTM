function proof()



%% Two agents in conflict only
numVehicles = 2;
U = createFeasibleSet(numVehicles);

% 1 has lower safety value
disp('Case 12')
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

[Uopt, Vopt, Iopt] = getOptSoln(C, U);
for i = 1:length(Uopt)
  disp('Uopt = ')
  disp(Uopt{i});
end

%% Three agents in conflict
numVehicles = 3;
U = createFeasibleSet(numVehicles);

% Case 12, 23
disp('Case 12, 23')
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(2,3) = 2;
C(3,2) = 1;
[Uopt, Vopt, Iopt] = getOptSoln(C, U);
for i = 1:length(Uopt)
  disp('Uopt = ')
  disp(Uopt{i});
end

% Case 12, 32
disp('Case 12, 32')
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(3,2) = 2;
C(2,3) = 1;

[Uopt, Vopt, Iopt] = getOptSoln(C, U);
for i = 1:length(Uopt)
  disp('Uopt = ')
  disp(Uopt{i});
end

% Case 12, 13
disp('Case 12, 13')
C = -ones(numVehicles);
C(1,2) = 2+0.1;
C(2,1) = 1+0.2;

C(1,3) = 2+0.1;
C(3,1) = 1+0.3;

[Uopt, Vopt, Iopt] = getOptSoln(C, U);
for i = 1:length(Uopt)
  disp('Uopt = ')
  disp(Uopt{i});
end

% Case 12, 23, 31
disp('Case 12, 23, 31')
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(2,3) = 2;
C(3,2) = 1;

C(3,1) = 2;
C(1,3) = 1;

[Uopt, Vopt, Iopt] = getOptSoln(C, U);
for i = 1:length(Uopt)
  disp('Uopt = ')
  disp(Uopt{i});
end

% Case 12, 23, 13
disp('Case 12, 23, 13')
C = -ones(numVehicles);
C(1,2) = 2;
C(2,1) = 1;

C(2,3) = 1;
C(3,2) = 2;

C(1,3) = 2;
C(3,1) = 1;

[Uopt, Vopt, Iopt] = getOptSoln(C, U);
for i = 1:length(Uopt)
  disp('Uopt = ')
  disp(Uopt{i});
end
end

function [Uopt, Vopt, Iopt] = getOptSoln(C, U)
V = zeros(size(U));
for i = 1:length(U)
  V(i) = objVal(C, U{i});
end

Iopt = find(V==max(V));
Vopt = V(Iopt);
Uopt = U(Iopt);
end

function val = objVal(C, U)
val = C.*U;
val = sum(val(:));
end

function U = createFeasibleSet(numVehicles)

if numVehicles == 2
  inds = {[1 2]; [2 1]};
elseif numVehicles == 3
  inds = {[1 2]; [1 3]; [2 1]; [2 3]; [3 1]; [3 2]};
else
  error('Only 2 or 3 vehicles is allowed!')
end


% Count up all possibilities as a binary in string form
Ustr = cell(2^length(inds),1);
for k = 1:2^length(inds)
  Ustr{k} = dec2bin(k-1);
  
  while length(Ustr{k}) < length(inds)
    Ustr{k} = ['0' Ustr{k}];
  end
end

% Convert the strings to a control matrix
U = cell(2^length(inds),1);
for k = 1:length(Ustr)
  for j = 1:length(Ustr{k})
    U{k}(inds{j}(1), inds{j}(2)) = str2double(Ustr{k}(j));
  end
end

% Get rid of all infeasible controls that have sum_j U(i,j) > 1
for k = length(U):-1:1
  if any(sum(U{k},2) > 1)
    U(k) = [];
  end
end

% Get rid of all infeasible controls that have U(i,j) + U(j,i) > 1
for k = length(U):-1:1
  flag = false;
  for i = 1:size(U{k}, 1)
    for j = i:size(U{k},2)
      if U{k}(i,j) + U{k}(j,i) > 1
        flag = true;
      end
    end
  end
  
  if flag
    U(k) = [];
  end
end
end