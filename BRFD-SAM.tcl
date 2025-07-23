# ==============================================================================================
# TCL Script for the Bidirectional Rotational Friction Damper Simplified Numerical Model (BRFD-SAM)
# Author: Eleonora Grossi
# Purpose: Simulation of the BRFD hysteresis cycle using Static and Transient Analysis
# ----------------------------------------------------------------------------------------------
# Model Overview:
# - 3D, 6 DOF per node
# - Uses elasticBeamColumn, flatSliderBearing, and zeroLength elements
# - Includes both static (vertical loads) and dynamic (ground motion) analysis
# ==============================================================================================

# ==============================================================================================
# MODEL DEFINITION
# ==============================================================================================

# --- Initialize Model ---
wipe;                          # Clear previous model
model basic -ndm 3 -ndf 6;     # 3D model, 6 DOF per node

# --- Define Nodes --- 
# Format: node ID X Y Z - Dimensions in mm
# BRFD external nodes
node 100 -275.885 0.000 0.000 
node 200 275.885 0.000 0.000

# --- Boundary Conditions ---
# Format: fix nodeID dX dY dZ rX rY rZ - If 0 free, if 1 fixed
fix 100 1 1 1 1 1 1
fix 200 0 0 1 1 1 1

# --- Material Definitions: Main BRFD Properties ---
# Steel01 defines a uniaxial bilinear material object with post-yielding hardening
# Format: uniaxialMaterial Steel01 matTag yielding-Force Elstic-Stiffness post-yielding ratio
uniaxialMaterial Steel01 11 11000 227000 0.00005; 	# Material properties for the longitudinal component
uniaxialMaterial Steel01 12 6000 204300 0.00005; 	# Material properties for the longitudinal component

# --- Define ZeroLonght Link Elements ---
# ZeroLength Links ensures the bidirectional BRFD behaviour along X (longitudinal component) and Y (transverse component)
# Format: ID iNode jNode -mat dxMat dyMat dzMat rxMat ryMat rzMat -dir dX(1) dY(2) dZ(3) rX(4) rY(5) rZ(6) -orient vector_x_X vector_x_Y vector_x_Z vector_y_X vector_y_Y vector_y_Z
# vector_x_X vector_x_Y vector_x_Z vector in global-coordinate system indicating local x-axis
# vector_y_X vector_y_Y vector_y_Z vector in global-coordinate system indicating local y-axis
element zeroLength 1 100 200 -mat 11 12 -dir 1 2; 		# material 11 assigned to translations along X-direction, material 12 assigned to translations along Y-direction

# ==============================================================================================
# OUTPUTS AND RECORDERS DEFINITION
# ==============================================================================================

# --- Set Directory for Output Folder --- 
set DIR "Output_BRFD_SAM";
file mkdir $DIR;

# --- Recorders Definition ---
recorder Node -file $DIR/NodeR_BRFD_SAM.out -node 100 -dof 1 2 3 4 5 6 reaction; 	# Reaction forces recorder
recorder Node -file $DIR/NodeD_BRFD_SAM.out -node 200 -dof 1 2 3 4 5 6 disp; 		# Displacements recorder

# ==============================================================================================
# STATIC ANALYSIS FOR STABILITY
# ==============================================================================================

puts "STATIC ANALYSIS";

# --- Set Static  Analysis ---
set OOM [EleStiffnessOOM]
set penvalue [expr pow(10.0, $OOM+8)]
system BandGeneral;
constraints Penalty $penvalue $penvalue;
numberer Plain;
test NormDispIncr 0.0010000000000000 1000
algorithm NewtonLineSearch
integrator LoadControl  0.100000
analysis Static

# --- Adaptive solver loop ---
set i 1
set step 10
set ok 0
# Analyze command RETURNS: 0 successful <0 unsuccessful
while {$ok == 0 && $i <= $step} {
puts "Step $i , Solver: NewtonLineSearch ..."; algorithm NewtonLineSearch; set ok [analyze 1];
if {$ok != 0} {puts "Step $i , Solver: KrylovNewton ..."; algorithm KrylovNewton; set ok [analyze 1];};
if {$ok != 0} {puts "Step $i , Solver: Newton ..."; algorithm Newton; set ok [analyze 1];};
if {$ok != 0} {puts "Step $i , Solver: ModifiedNewton ..."; algorithm ModifiedNewton; set ok [analyze 1];};
if {$ok != 0} {puts "Step $i , Solver: Broyden ..."; algorithm Broyden; set ok [analyze 1];};
if {$ok != 0} {puts "Step $i , Solver: BFGS ..."; algorithm BFGS; set ok [analyze 1];};
incr i 1;
};

if {$ok == 0} {puts "Static analysis SUCCESSFULLY";} else {puts "Static analysis FAILED";};

loadConst -time 0.0;

# ==============================================================================================
# CYCLIC ANALYSIS
# ==============================================================================================

puts "CYCLIC ANALYSIS";
# --- Set time-steps and duration --- 
set dt 0.001;
set numSteps 4000;
set PatternTag 2;
set IDgmSeries 1;

# --- Set Ground Motions Info ---
timeSeries Trig 3 0.00 4.00 2.00 ; 	# Setting sinusoidal waveform startin at 0 seconds, ending at 4 seconds and with period 2 seconds
pattern MultipleSupport $PatternTag  {
	     groundMotion 1 Plain -disp 3 -factor 40;		# Definition of GroundMotion 1 as a displacement with scale factor 40
	     groundMotion 2 Plain -disp 3 -factor 0;		# Definition of GroundMotion 2 as a displacement with scale factor 0
	 	 imposedSupportMotion  200 1  1; 				# Application of GroundMotion 1 to nodeID 200 along X direction (degree-of-fridom 1)
	 	 imposedSupportMotion  200 2  2;				# Application of GroundMotion 2 to nodeID 200 along Y direction (degree-of-fridom 2)
	 };

# --- Set Transient Analysis ---	 
algorithm NewtonLineSearch
integrator HHT 0.90; # No dissipation
analysis Transient

# --- Adaptive solver loop ---
set tFinal [expr $numSteps*$dt]
set tCurrent [getTime]
set tPercentage [expr $tCurrent/$tFinal*100]
set ok 0
# Analyze command RETURNS: 0 successful <0 unsuccessful
while {$ok == 0 && $tCurrent < $tFinal} {
puts "Time $tCurrent , Solver: NewtonLineSearch ..., [expr $tCurrent/$tFinal*100]"; algorithm NewtonLineSearch 0.6; set ok [analyze 1 $dt];
if {$ok != 0} {puts "Step $tCurrent , Solver: KrylovNewton ..."; algorithm KrylovNewton; set ok [analyze 1 $dt];}
if {$ok != 0} {puts "Step $tCurrent , Solver: Newton ..."; algorithm Newton; set ok [analyze 1 $dt];}
if {$ok != 0} {puts "Step $tCurrent , Solver: ModifiedNewton ..."; algorithm ModifiedNewton; set ok [analyze 1 $dt];}
if {$ok != 0} {puts "Step $tCurrent , Solver: Broyden ..."; algorithm Broyden; set ok [analyze 1 $dt];}
if {$ok != 0} {puts "Step $tCurrent , Solver: BFGS ..."; algorithm BFGS; set ok [analyze 1 $dt];}
set tCurrent [getTime];
}
if {$ok == 0} {puts "Transient analysis SUCCESSFULLY";} else {puts "Transient analysis FAILED";}

loadConst -time 0.0;

exit;