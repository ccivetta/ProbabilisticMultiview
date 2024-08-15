# ProbabilisticMultiview
Probabilistic object tracking using multiview cameras

This repository was designed for data collection and analysis from a binocular system in the United States Naval Academy's (USNA) Vision Integration in Polymanual Experimental Robotics (VIPER) Lab. 

A detailed explanation of the methods used can be found in the following: 

C. A. Civetta, D. H. Costello and M. D. M. Kutzer, "Probabilistic Object Tracking Using Quantified Camera Uncertainty Parameters in a Binocular System," 2024 International Conference on Unmanned Aircraft Systems (ICUAS), Chania - Crete, Greece, 2024, pp. 136-143, doi: 10.1109/ICUAS60882.2024.10556911.

Methods and code can be modified to integrate different tools and setups. 

In the VIPER Lab, ensure all code is run with Motion Capture connected and the UR10 connected
	- For Motion Capture: open "Motive" on the desktop and run the command "roslaunch vrpn_client_ros vrpn_optitrack.launch" in the terminal from the UR desktop that runs Ubuntu
	- For the UR10, power on the robot, select "run program", run the program "URServerScript"

The process for using the repository from start to finish in the VIPER lab is as follows: 

1. Run an initial camera calibration. This can be done by running the "BinocularCalibration_DataCollect.m" from MATLAB. Best results come when the UR10 is moves to ~3.5m from the checkerboard. 

2. Run the script "reprojectCheckerboardPoints" to verify collected data is not wildly inaccurate. If it is, restart from step 1, or manually remove clearly innacurate data. 

3. Run the script "ReprojectCheckerboardPointsLive" to evaluate how well the calibration is. Use the Create Figure section at the bottom of the script to visualize performance. The Boolean variable live=true will display a live reprojection in real time. live=false will move the UR10 to 10 specific joint angles and capture 10 images. 

4. Run the script "SCRIPT_solveTransformsRefined" to refine calibration values. This will refine the calibration values 5 times, and run a 10 image reprojection error test at each interval. The refined values for each iteration are saved with the number of times it has been refined appended to the end of the file. The script will display which iteration had the lowest reprojection error. These are the values that are to be used going forward, so go into the files saves under "Saved Matrices" and rename all of the refined values of the given best refinement to remove the number added at the end. After this, all following steps will use that refined data. 

5. Run the script "uncertaintyDataCollection". This will collect images of the checkerboard and red ball in the same image. It will process all of the data and then display each image and prompt the user to select the pixel location of the red ball center in the image. To do this, first use the MATLAB zoom feature to zoom in on the ball. Then press the spacebar, and you will see crosshairs appear on the screen. Move the crosshairs to the center of the ball, and press spacebar again. Repeat for all images. 

6. Run the script "MonteCarlo_OverallUncertainty"

7. Run the script "ReprojectBall_UncertaintyAwareVisualization"



