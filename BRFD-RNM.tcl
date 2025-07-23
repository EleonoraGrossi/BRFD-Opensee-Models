# ==============================================================================================
# TCL Script for the Bidirectional Rotational Friction Damper Refined Numerical Model (BRFD-RNM)
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
# Group 1: Core and side plates nodes
node 110 -155.885 0.000 -14.000 
node 111 -121.816 0.000 -14.000 
node 112 -189.954 0.000 -14.000 
node 120 -155.885 0.000 0.000 
node 121 -121.816 0.000 0.000 
node 122 -189.954 0.000 0.000 
node 130 -155.885 0.000 14.000 
node 131 -121.816 0.000 14.000 
node 132 -189.954 0.000 14.000 
node 210 0.000 -90.000 -14.000 
node 211 34.069 -90.000 -14.000 
node 212 -34.069 -90.000 -14.000 
node 220 0.000 -90.000 0.000 
node 221 34.069 -90.000 0.000 
node 222 -34.069 -90.000 0.000 
node 230 0.000 -90.000 14.000 
node 231 34.069 -90.000 14.000 
node 232 -34.069 -90.000 14.000 
node 310 0.000 90.000 -14.000 
node 311 34.069 90.000 -14.000 
node 312 -34.069 90.000 -14.000 
node 320 0.000 90.000 0.000 
node 321 34.069 90.000 0.000 
node 322 -34.069 90.000 0.000 
node 330 0.000 90.000 14.000 
node 331 34.069 90.000 14.000 
node 332 -34.069 90.000 14.000 
node 410 155.885 0.000 -14.000 
node 411 189.954 0.000 -14.000 
node 412 121.816 0.000 -14.000 
node 420 155.885 0.000 0.000 
node 421 189.954 0.000 0.000 
node 422 121.816 0.000 0.000 
node 430 155.885 0.000 14.000 
node 431 189.954 0.000 14.000 
node 432 121.816 0.000 14.000
node 510 0.000 0.000 0.000 
# Group 2: Connection plates external nodes
node 600 -275.885 0.000 0.000 
node 700 275.885 0.000 0.000
# Group 3: Alignment guides nodes
node 501 0.000 0.000 -28.000 
node 502 0.000 0.000 28.000
node 801 -155.885 0.000 -28.000 
node 802 -155.885 0.000 28.000 
node 901 155.885 0.000 -28.000 
node 902 155.885 0.000 28.000

# --- Boundary Conditions ---
# Format: fix nodeID dX(1) dY(2) dZ(3) rX(4) rY(5) rZ(6) - If 0 free, if 1 fixed
fix 600 1 1 1 1 1 1;		# Fixed in all directions
fix 700 0 0 1 1 1 1;		# Partially fixed: allows horizontal translation (X,Y)

# --- Material Definitions ---
uniaxialMaterial Elastic 20 1e+10;	# Stiff material
uniaxialMaterial Elastic 21 1e-06; 	# Soft material
frictionModel Coulomb 22 0.400000;	# Friction coefficient for sliding: Coulomb Friction model

# --- Constraints: linking nodes across vertical layers ---
# EqualDOF ensures core central and side external nodes to move together in X and Y
# Format: equalDOF RetainerNodeID ConstrainedNodeID dX(1) dY(2) dZ(3) rX(4) rY(5) rZ(6) - Write only the degree-of-freedom number to be constrained
equalDOF 120 110 1 2 
equalDOF 120 130 1 2 
equalDOF 220 210 1 2 
equalDOF 220 230 1 2 
equalDOF 320 310 1 2 
equalDOF 320 330 1 2 
equalDOF 420 410 1 2 
equalDOF 420 430 1 2 
equalDOF 510 501 1 2 
equalDOF 510 502 1 2
# ZeroLength Links ensures alignment guide external nodes to move together in X,Y and allow rotations around Z
# Format: ID iNode jNode -mat dxMat dyMat dzMat rxMat ryMat rzMat -dir dX(1) dY(2) dZ(3) rX(4) rY(5) rZ(6) -orient vector_x_X vector_x_Y vector_x_Z vector_y_X vector_y_Y vector_y_Z
# vector_x_X vector_x_Y vector_x_Z vector in global-coordinate system indicating local x-axis
# vector_y_X vector_y_Y vector_y_Z vector in global-coordinate system indicating local y-axis
element zeroLength 1152 110 801 -mat 20 20 21 21 20 20 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1153 130 802 -mat 20 20 21 21 20 20 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1154 410 901 -mat 20 20 21 21 20 20 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1155 430 902 -mat 20 20 21 21 20 20 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00

# --- Geometry Transformation ---
# Format: geomTransf Corotational tagID vector_coord_X vector_coord_Y vector_coord_Z - Local axes transformation for beam elements 
geomTransf Corotational 1 0 0 1;	# The local system of the beam elements have the local z-axis coincident with the global Z-axis

# --- Define Beam Elements ---
# Format: element elasticBeamColumn tagID iNode jNode A E G J Iy Iz transfTag
# Units assumed: SI (e.g., N, mm)
# Group 1: Core, side and connection plates and alignment guides beam elements
element elasticBeamColumn 100 110 210 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 101 130 230 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 102 220 510 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 103 510 320 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 104 310 410 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 105 330 430 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 106 600 120 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 107 420 700 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 108 801 501 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 109 802 502 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 110 501 901 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 111 502 902 100 210000 80769 200000000 100000000 100000000 1
# Group 2: Subassembly 1 beam elements for Dissipative Area 1
element elasticBeamColumn 112 110 111 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 113 110 112 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 114 120 121 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 115 120 122 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 116 130 131 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 117 130 132 100 210000 80769 200000000 100000000 100000000 1 
# Group 3: Subassembly 2 beam elements for Dissipative Area 2
element elasticBeamColumn 118 210 211 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 119 210 212 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 120 220 221 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 121 220 222 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 122 230 231 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 123 230 232 100 210000 80769 200000000 100000000 100000000 1
# Group 4: Subassembly 3 beam elements for Dissipative Area 3 
element elasticBeamColumn 124 310 311 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 125 310 312 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 126 320 321 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 127 320 322 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 128 330 331 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 129 330 332 100 210000 80769 200000000 100000000 100000000 1
# Group 5: Subassembly 4 beam elements for Dissipative Area 4 
element elasticBeamColumn 130 410 411 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 131 410 412 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 132 420 421 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 133 420 422 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 134 430 431 100 210000 80769 200000000 100000000 100000000 1 
element elasticBeamColumn 135 430 432 100 210000 80769 200000000 100000000 100000000 1

# --- Flat Slider Bearings ---
# Flat slider bearings simulating the rotational friction interface
# Format: element flatSliderBearing ID iNode jNode fricTag KInit -P axialMat -T torsionalMat -My y-momentMat -Mz z-momentMat -orient vector_x_X vector_x_Y vector_x_Z vector_y_X vector_y_Y vector_y_Z -iter max_iter_number tollerance
# vector_x_X vector_x_Y vector_x_Z vector in global-coordinate system indicating local x-axis
# vector_y_X vector_y_Y vector_y_Z vector in global-coordinate system indicating local y-axis
# Group 1: Flat Slider Bearings for Dissipative Area 1
element flatSliderBearing 1136 121 111 22 200000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1137 122 112 22 200000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1138 121 131 22 200000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1139 122 132 22 200000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10
# Group 2: Flat Slider Bearings for Dissipative Area 2
element flatSliderBearing 1140 221 211 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1141 222 212 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1142 221 231 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1143 222 232 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10
# Group 3: Flat Slider Bearings for Dissipative Area 3 
element flatSliderBearing 1144 321 311 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1145 322 312 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1146 321 331 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1147 322 332 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10
# Group 4: Flat Slider Bearings for Dissipative Area 4
element flatSliderBearing 1148 421 411 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1149 422 412 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1150 421 431 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10 
element flatSliderBearing 1151 422 432 22 100000 -P 20 -T 21 -My 20 -Mz 20 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 -iter 60 10.0e-10

# --- Apply Static Vertical Loads --- 
# Positive and negative vertical forces applied on Flat Slider Bearings to simulate bolts pre-loading 
pattern Plain 1 Constant { 
	load 111 0 0 16500 0 0 0;	# Lower Dissipative Area 1 points
	load 112 0 0 16500 0 0 0;	# Lower Dissipative Area 1 points
	load 131 0 0 -16500 0 0 0;	# Upper Dissipative Area 1 points
	load 132 0 0 -16500 0 0 0;	# Upper Dissipative Area 1 points
	load 211 0 0 16500 0 0 0;	# Lower Dissipative Area 2 points
	load 212 0 0 16500 0 0 0;	# Lower Dissipative Area 2 points
	load 231 0 0 -16500 0 0 0;	# Upper Dissipative Area 2 points
	load 232 0 0 -16500 0 0 0;	# Upper Dissipative Area 2 points
	load 311 0 0 16500 0 0 0;	# Lower Dissipative Area 3 points
	load 312 0 0 16500 0 0 0;	# Lower Dissipative Area 3 points
	load 331 0 0 -16500 0 0 0;	# Upper Dissipative Area 3 points
	load 332 0 0 -16500 0 0 0;	# Upper Dissipative Area 3 points
	load 411 0 0 16500 0 0 0;	# Lower Dissipative Area 4 points
	load 412 0 0 16500 0 0 0;	# Lower Dissipative Area 4 points
	load 431 0 0 -16500 0 0 0;	# Upper Dissipative Area 4 points
	load 432 0 0 -16500 0 0 0;	# Upper Dissipative Area 4 points
};

# ==============================================================================================
# OUTPUTS AND RECORDERS DEFINITION
# ==============================================================================================

# --- Set Directory for Output Folder --- 
set DIR Output_BRFD_RNM;
file mkdir $DIR;

# --- Recorders Definition --- 
recorder Node -file $DIR/NodeR_BRFD_RNM.out -node 600 -dof 1 2 3 4 5 6 reaction; 	# Reaction forces recorder
recorder Node -file $DIR/NodeD_BRFD_RNM.out -node 700 -dof 1 2 3 4 5 6 disp; 		# Displacements recorder

# ==============================================================================================
# FIRST STATIC ANALYSIS FOR BOLTS' VERTICAL LOAD
# ==============================================================================================

puts "STATIC ANALYSIS";
set OOM [EleStiffnessOOM]
set penvalue [expr pow(10.0, $OOM+8)]
system BandGeneral;
constraints Penalty $penvalue $penvalue;
numberer Plain;
test NormDispIncr 0.0010000000000000 1000
algorithm NewtonLineSearch
integrator LoadControl  0.100000
analysis Static
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
# ADDITIONAL CONSTRAINTS FOR STABILITY
# ==============================================================================================

# ZeroLength Links to improve steadiness of bolts pre-load
# Format: ID iNode jNode -mat dxMat dyMat dzMat rxMat ryMat rzMat -dir dX(1) dY(2) dZ(3) rX(4) rY(5) rZ(6) -orient vector_x_X vector_x_Y vector_x_Z vector_y_X vector_y_Y vector_y_Z
# vector_x_X vector_x_Y vector_x_Z vector in global-coordinate system indicating local x-axis
# vector_y_X vector_y_Y vector_y_Z vector in global-coordinate system indicating local y-axis
uniaxialMaterial Elastic 24 1e+12;	# Highly-stiff material
element zeroLength 1156 121 111 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1157 122 112 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1158 121 131 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1159 122 132 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1160 221 211 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1161 222 212 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1162 221 231 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1163 222 232 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1164 321 311 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1165 322 312 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1166 321 331 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1167 322 332 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1168 421 411 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1169 422 412 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 -1.00 0.00 1.00 0.00 
element zeroLength 1170 421 431 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00 
element zeroLength 1171 422 432 -mat 24 21 21 21 21 21 -dir 1 2 3 4 5 6 -orient 0.00 0.00 1.00 0.00 -1.00 0.00

# ==============================================================================================
# SECOND STATIC ANALYSIS FOR STABILITY
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
	 	 imposedSupportMotion  700 1  1; 				# Application of GroundMotion 1 to nodeID 700 along X direction (degree-of-fridom 1)
	 	 imposedSupportMotion  700 2  2;				# Application of GroundMotion 2 to nodeID 700 along Y direction (degree-of-fridom 2)
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
