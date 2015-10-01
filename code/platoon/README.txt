Code folder for the platooning.

Classes:
- Folders beginning with @ contains code for each of the classes used. These classes include 10Dquadrotor (unused), linpath, highway (which inherits from linpath), platoon, and quadrotor (should be inherited from a vehicle class, to be added).

Simulation code:
- Run any of the simulation code to see simulations of example scenarios
- For starters, check out simulateNormal2.m, which shows two quadrotors merging onto a highway. 
- Other simulations are found in simulateNormal.m, simulateHighwayMerge.m, simulateIntruder.m, simulateFaulty.m, and some others

Reachability computations:
- these are for computing the look-up tables (.mat files in the code/ folder)required for the reachability-based controllers that the simulation code uses