addpath ../..

clear all
pause('on');

X1_0 = 0;
Y1_0 = 0;
THETA1_0 = 0;
V1_0 = 5;

X2_0 = 45;
Y2_0 = 0;
THETA2_0 = pi;
V2_0 = 5;

T_MAX = 7; %time length for evolution of simulation
DELTA_T = .1;

RADIUS = 5;
V_A = 5;
V_B = 5;
MAX_A = 1;
MAX_B = 1;

load datafile.mat
load gfile.mat

reachInfo.uMax = 1;
reachInfo.uMin = -1;
reachInfo.vMax = 1;
reachInfo.vMin = -1;
plane1 = plane(1, [20; 0; -pi/4; V1_0], reachInfo);
plane2 = plane(2, [40; -20; 3*pi/4; V2_0], reachInfo);

costates = extractCostates(g, data);

figure
axis([0 50 -10 30]);
collided = false;
plane1.dt = DELTA_T;
plane2.dt = DELTA_T;
for t = 0:DELTA_T:T_MAX
  plane.plotPositions({plane1, plane2}, [0 50 -30 10]);
  
  z = plane1.getRelativeStates(plane2);  
  z = z(1,:);
  z = z(1:2);  

  if collided
    scatter(col_X, col_Y, 300, [1 0 0], '*');  
  elseif isCollided(plane1, plane2, RADIUS)
    collided = true;
    p1_pos  = plane1.getPosition();
    p2_pos = plane2.getPosition();
    col_X = [p1_pos(1) p2_pos(1)];
    col_Y = [p1_pos(2) p2_pos(2)];
  end
  
  safeV.g = g;
  safeV.data = data;  
  [safe, uSafe, valuex] = plane1.isSafe({plane2}, safeV, costates);    
  safe = safe(1);
  uSafe = uSafe(1);
  valuex = valuex(1);
  if ~safe
    u_1 = uSafe;
  else
    u_1 = 0; 
  end    
  plane1.updateState([u_1 plane1.u(2)]);
  
  u_2 = 0;
  %{
 can use this code to make plane2 actually follow plane1 (maybe not
   optimally)
  ang = atan2(plane1.x(2)-plane2.x(2),plane1.x(1)-plane2.x(1));
  if ang > plane2.x(3)
    u_2 = 1;
  else
    u_2 = -1;
  end
  %}
  plane2.updateState([u_2 plane2.u(2)]);
  pause(.1);
end