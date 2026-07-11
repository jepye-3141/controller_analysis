# 6-DOF Ballistics Simulation

> MATLAB Simulation of a six degree of freedom (6-DOF) ballistic flight of a 120mm mortar round.

![Sample Output](/images/output.png)

![Sample Output](/images/console_output.png)

## Files

The simulation is setup, run, and controlled from the Mortar_Sim.m file. This main file uses the ODE45 solver in MATLAB with the equations of motion (EoM) specified for each time step in the EoM.m file. The file reads input data from a standard atmosphere table and a table of aerodynamic coefficients vs. Mach number (McCoy, 1998). The 6-DOF model was developed based on the methodology in McCoy, chapter 9.

1. [Mortar_Sim.m](Mortar_Sim.m) - Simulation holder with some sample initial values
2. [aero_constants.m](aero_constants.m) - Unpacking and calculation of aerodynamic constants from the provided Aerodynamic Characteristics file
3. [std_atm.csv](std_atm.csv) - Table of the standard atmosphere
4. [Aerodynamic_Char_120mm_Mortar.xlsx](Aerodynamic_Char_120mm_Mortar.xlsx) - Aerodynamic coefficients of the 120mm for different Mach number. Data extracted from McCoy, 1998 on page 220.

## References

1. McCoy, RL, Modern Exterior Ballistics: The Launch and Flight Dynamics of Symmetric Projectiles, Schiffer Military History, Atglen, PA, 1998.
