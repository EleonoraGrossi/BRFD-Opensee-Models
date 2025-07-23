# BRFD-Opensee-Models
Bidirectional Rotational Friction Damper Simplified and Refined Numerical Models (BRFD-SAM & BRFD-RNM) in OpenSees

DESCRIPTION
-----------
This file documents two TCL scripts that defines the a 3D refined and a simplified numerical models of a Bidirectional Rotational Friction Damper (BRFD-RNM & BRFD-SAM). The BRFD-RNM uses Flat Slider Bearing and Elastic Beam Column elements, adopting a Corotational transformation to reproduce circular motion , while the BRFD-SAM uses Zero-Length Link elements incorporating two orthogonal Steel01 materials (one for each component). Both models are intended for analysis using OpenSees.

The BRFD-RNM model includes the BRFD elements and internal constraints, supporting both static and dynamic analyses.

The BRFD-RNM script creates:
- A 3D BRFD model with nodes and beam elements
- Flat slider bearings modelled using "flatSliderBearing" elements to simulate a rotational friction interface
- EqualDOF and ZeroLength links for simulating internal constraints
- A complete static and transient analysis workflow
- Recorders for displacements and reactions

The BRFD-SAM model reproduces the BRFD behaviour, supporting both static and dynamic analyses.

The BRFD-SAM script creates:
- A 3D simplified BRFD model with nodes and ZeroLength links elements
- Steel01 uniaxial materials are used to simulate BRFD components
- A complete static and transient analysis workflow
- Recorders for displacements and reactions

FILES & DIRECTORY STRUCTURE
-----------------------------------------------------
	/BRFD-Opensee-Models/
	|__ /Output_BRFD_RNM/		- Automatically created folder with BRFD-RNM output data
	|__ /Output_BRFD_SAM/		- Automatically created folder with BRFD-SAM output data
	|__  BRFD-RNM.png			- BRFD-RNM model image in .png
	|__  BRFD-RNM.tcl			- Main BRFD-RNM TCL script
	|__  BRFD-SAM.tcl			- Main BRFD-SAM TCL script
	|__  README.txt				- This file

HOW TO RUN THE MODELS
-----------------------------------------------------
1. Install OpenSees (https://opensees.berkeley.edu)
2. Open a terminal and navigate to the folder containing the script
3. Run the following commands:

   	- OpenSees BRFD-RNM.tcl
	Output files will be written to the /Output_BRFD_RNM/ folder.

   or

   	- OpenSees BRFD-SAM.tcl
	Output files will be written to the /Output_BRFD_SAM/ folder.   

OUTPUT FILES
-----------------------------------------------------
Key output files include:

- NodeR_BRFD_RNM.out      - Reaction forces at fixed node of BRFD-RNM
- NodeD_BRFD_RNM.out      - Displacements at the free node of BRFD-RNM

or

- NodeR_BRFD_SAM.out      - Reaction forces at fixed node of BRFD-SAM
- NodeD_BRFD_SAM.out      - Displacements at the free node of BRFD-SAM

ANALYSIS TYPES IN BRFD-RNM
-----------------------------------------------------
1. STATIC ANALYSIS
   - Vertical loads applied to rotational friction interface external nodes to simulate bolts pre-load
   - Multiple solvers tried automatically in case of convergence issues

2. TRANSIENT ANALYSIS
   - HHT integration method used for dynamic loading
   - Optional cyclic or ground motion applied using imposed support motion

ANALYSIS TYPES IN BRFD-SAM
-----------------------------------------------------
1. STATIC ANALYSIS
   - Multiple solvers tried automatically in case of convergence issues

2. TRANSIENT ANALYSIS
   - HHT integration method used for dynamic loading
   - Optional cyclic or ground motion applied using imposed support motion

CUSTOMIZATION NOTES
-----------------------------------------------------
- Node geometry and coordinates can be modified to change the system layout
- Material properties (elastic, friction) are easily tunable
- Time series for ground motion can be replaced with custom data
- Time step and duration for transient analysis can be adjusted in:
     set dt
     set numSteps

CONTACT
-----------------------------------------------------
For questions, issues, or contributions, feel free to reach out via GitHub or email:
eleonora.grossi@unife.it

For comprehensive details about the BRFD system and the implemented models, please refer to the following open-access journal articles:
1.  Grossi E., De Risi R., Zerbin M., De Luca F., Aprile A. “Refined numerical modelling of a bidirectional rotational friction damper (BRFD) for seismic retrofitting” (submitted for publication)
2. Grossi E., De Risi R., Zerbin M., De Luca F., Aprile A. (2025) “A Novel Bidirectional Friction Damper for Retrofitting of RC Precast Structures: Experimental and Numerical Assessment”, Engineering Structures 339 (120529). DOI: 10.1016/j.engstruct.2025.120529
3. Grossi E., Zerbin M., Aprile A., De Risi R., De Luca F. (2024) “Conceptual study of an innovative friction damper for the seismic retrofit of precast RC structures with poor connections”, Structures 67 (106960). DOI: 10.1016/j.istruc.2024.106960
