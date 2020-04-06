README File
_________________________________________________________________________________________

				Project 3 | Phase 3 | ENPM 661 | Planning for Autonomous Robots |
						A* Programme for Non-Holonomic robot

						SUBMISSION : April 10, 2020

--------------------------------------------------%%-------------------------------------------<br/>
GITHUB:  https://github.com/govindak-umd/ENPM661/tree/master/Project%203/Project%203%20Phase%203 <br/>
YOUTUBE: 
--------------------------------------------------%%-------------------------------------------<br/>
<br/>
<br/>
![](Images/Obstacle_Space.jpg)<br/>
<br/>
<br/>
![](Images/Backtracked_nodes.jpg)<br/>
<br/>
<br/>
![](Images/Visited_nodes.jpg)<br/>
<br/>
<br/>
![](Images/Quiver_Map.png)<br/>
<br/>
<br/>
_________________________________________________________________________________________
Authors: <br/>

Govind Ajith Kumar<br/>
UID : 116699488<br/>

Masters Robotics 
University of Maryland<br/>
College Park<br/>
Maryland<br/>
20740 USA<br/>
--------------------------
Rajeshwar NS<br/>
UID : 116921237<br/>

Masters Robotics<br/>
University of Maryland<br/>
College Park<br/>
Maryland<br/>
20740 USA<br/>
_________________________________________________________________________________________
Language: Python 3.7.x<br/>

THIS PROGRAMME TAKES THE USER INPUTS BELOW AND SOLVES THE OBSTACLE SPACE VERY QUICKLY<br/>
time taken (for A* solving) ~ <1 second<br/>
To solve the necessary test cases, please know that the start and the goal points here are adjusted based on the clearance and the width of the robot<br/>
This avoids the goal or the start getting into the borders or any obstacles<br/>
For the turtle bot, this value is fixed at 35<br/>
If you wish to change this value, please rerun the code after doing so.<br/>
ALSO !! NOTE !!, the coordinates of the obstacle points have been multiplied by a factor of 100<br/>

This ensures a clearer graph. Hence, please multiply by a factor of 100, before input. For example: 5.5 will be = 5.5*100 = 550<br/>
Range of x coordinates you can enter  =  0 - 1020<br/>
Range of y coordinates you can enter  =  0 - 1020<br/>
--------------
Please wait while the map is being generated. Approximate wait time < 5 seconds . <br/>
-------------
Please wait while your solution is being calculated . <br/>
Time taken on a high end Personal Computer ~ < 1 second<br/>
--------------
This will generate a map with solution, backtracked path, quiver plot and an animation too!<br/> 
NOTE: Quiver plot will take an additional 10 minutes to calculate <br/>

_________________________________________________________________________________________
Libraries implemented and required to be installed for the code to execute:<br/>

numpy :: for matrices and other associated numerical calculations<br/>
copy :: to copy<br/>
math :: for all mathematical operations<br/>
heapq :: for priority queue<br/>
time :: for measuring the time<br/>
cv2 :: for image related operations (OpenCV 4.xx)<br/>
pygame :: For animations<br/>
matplotlib:: To plot the graphs and see the path taken by the robot from the start to the goal node and the quiver plots<br/>

			NOTE :::: PLEASE MAKE SURE THAT THE MATPLOTLIB GRAPHS CAN BE DISPLAYED, ESPECIALLY IF BEING RUN ON SPYDER or even any other IDE
_________________________________________________________________________________________

Code : Solves the obstacle map based on the A* Algorithm For Rigid Robot with a radius and clearance fro non- Holonomic Drive Robot<br/>
<br/>
The following inputs are required by the user:<br/>

-> Vector with x and y coordinate of the start and the initial orientation<br/>
-> Vector with x and y coordinate of the goal<br/>
-> LEFT WHEEL RPM AND RIGHT WHEEL RPM<br/>
-> Step Count<br/>
-> time you wish to choose to run the robot<br/>
Enter the  here, with a blank in between them (suggested : 10 10 30) : > 10 10 30<br/>
Enter the x and y coordinate of the goal and the , in the form of blanks (suggested : 1010 1010) : > 1010 1010<br/>
Enter the LEFT WHEEL RPM AND RIGHT WHEEL RPM  : > 3 8<br/>
Enter the clearance of the robot with the nearby obstacles : (suggested value  < 5) : > 5<br/>
Enter the step (1-10): (suggested values  = 1) : > 1<br/>
Please enter the time you wish to choose to run the robot (suggested value = 1 second) : 1<br/>


---------------------------------------------------------------------------------------------
Time for solution for the following inputs: <br/>
approx 5 seconds for map generation<br/>
1 second for solution<br/>
2 seconds for visited and backtracked maps<br/>
<10 seconds for quiver plot<br/>
##############################################################################################

Enter the x and y coordinate of the start and the initial orientation here, with a blank in between them (suggested : 10 10 30) : > 10 10 30<br/>
Enter the x and y coordinate of the goal and the , in the form of blanks (suggested : 1010 1010) : > 1010 1010<br/>
Enter the LEFT WHEEL RPM AND RIGHT WHEEL RPM  : > 3 8<br/>
Enter the clearance of the robot with the nearby obstacles : (suggested value  < 5) : > 5<br/>
Enter the step (1-10): (suggested values  = 1) : > 1<br/>
Wheel radius is calculated to be 76mm/2 -> approximating to 0.5<br/>
Please enter the time you wish to choose to run the robot (siggested value = 1 second) : 1<br/>
##############################################################################################


Both codes takes a few minutes to solve (subject to the system specifications), and then output 
the following:<br/>

##After Reaching the goal, the backtracking will commence.<br/>

## Once the backtracking is complete the following sets of outputs are given out:<br/>

> Image of the obstacles (OpenCV)<br/>

> Animation showing the visited nodes and the backtracked path (using pygame)<br/>

> Graph plot showing path of solution (MatplotLib)<br/>

> Image of the visited nodes (OpenCV)<br/>

> Image of backtracked path (OpenCV)<br/>

> Graph plot showing quiver plot(MatplotLib)<br/>

---------------------------------------------------------------------------------------------



