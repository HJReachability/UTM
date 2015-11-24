===============================
Code folder for the platooning
===============================

(Also see documentation in each file.) 

===== Classes =====
- Folders beginning with @ contains code for each of the classes used. These classes include 10Dquadrotor (unused), linpath, highway (which inherits from linpath), platoon, and quadrotor (should be inherited from a vehicle class, to be added).

===== Simulation code =====
- Run any of the simulation code to see simulations of example scenarios
- For starters, check out simulateNormal2.m, which shows two quadrotors merging onto a highway. 
- Other simulations are found in simulateNormal.m, simulateHighwayMerge.m, simulateIntruder.m, simulateFaulty.m, and some others
- simulation depends on the saved reachable set

Reachability computations:
- these are for computing the look-up tables (.mat files in the code/ folder)required for the reachability-based controllers that the simulation code uses

Other code:
- the code here depends on the levelset toolbox and the helper library. Links to the repositories:

Level Set Toolbox:
http://www.cs.ubc.ca/~mitchell/ToolboxLS/

Helper:
https://repo.eecs.berkeley.edu/git/users/mochen72/helperOC.git