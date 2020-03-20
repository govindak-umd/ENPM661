README File
_________________________________________________________________________________________

Project 3 Phase 2 - ENPM 661 - Planning for Autonomous Robotics
_________________________________________________________________________________________
Authors: 

Govind Ajith Kumar
UID : 116699488

Masters Robotics 
University of Maryland
College Park
Maryland
20740 USA
--------------------------
Rajeshwar NS
UID : 116921237

Masters Robotics
University of Maryland
College Park
Maryland
20740 USA
_________________________________________________________________________________________
Language: Python 3.7.x
_________________________________________________________________________________________
Libraries implemented and required to be installed for the code to execute:

numpy :: for matrices and other associated numerical calculations. 
math :: for all mathematical operations
heapq :: for priority queue
time :: for measuring the time
cv2 :: for image related operations (OpenCV 4.xx)
pygame :: For animations
matplotlib:: To plot the graphs and see the path taken by the robot from the start to the goal node
_________________________________________________________________________________________

This README file is for two codes
The programme solves the obstacle map based on the Dijkstras Algorithm. 

Code : Solves the obstacle map based on the A* Algorithm For Rigid Robot with a radius and clearance

The following inputs are required by the user:

x_start= the x coordinate of the start
y_start= y coordinate of the start
orientation = starting orientation of the Robot
x_goal= x coordinate of the goal
y_goal= y coordinate of the goal
Radius = Radius of the robot
Clearence = Clearance of the robot


---------------------------------------------------------------------------------------------

Both codes takes a few minutes to solve (subject to the leptop specifications), and then output 
the following:

>After Reaching the goal, the backtracking will commence.

> Once the backtracking is complete the following sets of outputs are given out.

> Image of the obstacles

> The path backtracked by the code.


---------------------------------------------------------------------------------------------



