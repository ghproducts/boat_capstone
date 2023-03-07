Computer Vision Boat Docking
Design of Computational Subsystem and Elements
Gavin Hearne┃gavinhearne@gwu.edu

The computation subsystem consists of code written to process visual data, calculate optimal paths, and generate motor speeds and rudder angles so that the boat can follow the calculated path. An overview for how this is performed is shown in Figure 1.


Figure 1. Computation flowchart

When activated, the compute unit (in this case a standard laptop) receives a video signal from the boat-mounted camera. Before any positional calculations can be performed, the signal must be calibrated to account for camera distortion so that the 2-D signal can be used to accurately position the camera in a 3-D coordinate plane. This is done beforehand through the use of python libraries published by openCV, which contain the calibration and computer vision algorithms used to detect arUco markers. The calibration algorithm is first provided with examples of a known pattern, generally in the form of a video recording using the desired camera optic of an accurately measured flat checkerboard. The software then breaks the footage down into separate frames and finds the position and warping of the chessboard image in each one. From this, the algorithm can generate a matrix that, when multiplied by a camera input with the same optical properties, will generate a signal that can be used to accurately measure distance and position. 
When the calibrated signal is passed on to further sections of the program, it then performs a quick check to determine whether an arUco marker is within the camera frame. If it is detected, then the positioning of the marker is found using openCV pose estimation, which outputs the rotational and translational vectors of the marker relative to the camera. This can, through vector transformations, be converted into the position of the camera, some results of which are shown in figure 2. It is important to note that the size of the arUco marker must be predefined and provided to the software beforehand, else the accuracy of the positioning will be significantly impacted. 

Figure  2. OpenCV camera positioning example

After the boat position is calculated, a path between the boat and the final docking position is calculated. This is performed using the Dubins path algorithm from the PythonRobotics library, which connects initial and final conditions together in a 2-D plane with a constraint placed on the max curvature the path can take. The original paper published by L.E. Dubins[1] discusses in more detail how this algorithm works, and figure 3 shows two examples of this algorithm in use, with a random starting state and an end state set to be parallel to the x-axis at (0,0).  


Figure 3. Control system simulation
Figure 3 also demonstrates a simulation of the boat motion with a proportional control scheme applied. The control system is a slightly modified proportional control system, which takes the position error between the generated path and a position directly in front of the boat, as shown in Figure 4. This error is multiplied by a constant K and then set as the rudder angle for the boat to follow next. This instruction set is then sent to the arduino electrics control unit. To assist in the development of the control scheme prior to the construction of the boat, a simulation based on the riverboat dynamics model described by Aleksander GRM[2] was developed in python. This simulation assists in demonstrating the capabilities of different methods and algorithms, and can aid in tuning the different constants necessary for the proper function of the control system. This is demonstrated as the “simulated boat 		Figure 4. Control system error
path” line in figures 3 and 4.								



References:				
[1] Dubins, L. E. (1957). On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents. American Journal of mathematics, 79(3), 497-516

[2] Grm, A. (2017). Mathematical model for riverboat dynamics. Brodogradnja: Teorija i praksa brodogradnje i pomorske tehnike, 68(3), 25-35.



.

		
