README File
_________________________________________________________________________________________

				Project 3 | Phase 4 | ENPM 661 | Planning for Autonomous Robots |
						A* Programme OBSTACLE SOLUTION
						on TurtleBot using ROS in Python 2

						SUBMISSION : April 10, 2020

--------------------------------------------------%%-------------------------------------------
	
	GITHUB: 

Initial repo with many old commits : 


https://github.com/govindak-umd/ENPM661/tree/master/Project%203/Project%203%s20Phase%204


Newly created repo just for the project:


--------------

	YOUTUBE:


 
Two videos have been linked :




Case 1 -  PERTAINING TO CODE_1.py
https://www.youtube.com/watch?v=KAAa5h_2YO0&t=1s

Case 2 -  PERTAINING TO CODE_2.py
https://www.youtube.com/watch?v=pjIAvvLftzQ

--------------------------------------------------%%-------------------------------------------

Codes to execute for Code_1 : 

# catkin_make
# source ./devel/setup.bash 
# ROBOT_INITIAL_POSE="-x -4.30 -y -3.00" roslaunch tt_gazebo tt_launch.launch
# rosrun tt_gazebo code_1.py 


Codes to execute for Code_2 : 

# catkin_make
# source ./devel/setup.bash 
# ROBOT_INITIAL_POSE="-x -4.30 -y -3.80" roslaunch tt_gazebo tt_launch.launch
# rosrun tt_gazebo code_2.py 

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

THIS PROGRAMME TAKES THE USER INPUTS BELOW AND SOLVES THE OBSTACLE SPACE VERY QUICKLY
time taken (for A* solving) ~ <1 second
To solve the necessary test cases, please know that the start and the goal points here are adjusted based on the clearance and the width of the robot
This avoids the goal or the start getting into the borders or any obstacles
For the turtle bot, this value is fixed at 35
If you wish to change this value, please rerun the code after doing so.
ALSO !! NOTE !!, the coordinates of the obstacle points have been multiplied by a factor of 100

This ensures a clearer graph. Hence, please multiply by a factor of 100, before input. For example: 5.5 will be = 5.5*100 = 550
Range of x coordinates you can enter  =  0 - 1020
Range of y coordinates you can enter  =  0 - 1020
-------------
Please wait while your solution is being calculated . 
Time taken on a high end Personal Computer ~ < 1 second
--------------
_________________________________________________________________________________________
Libraries implemented and required to be installed for the code to execute:



-----ROS------
rospy :: for ros
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Int16
--------------

Other Python Libraries:

numpy :: for matrices and other associated numerical calculations
copy :: to copy
math :: for all mathematical operations
heapq :: for priority queue
time :: for measuring the time
cv2 :: for image related operations (OpenCV 4.xx)
pygame :: For animations
matplotlib:: To plot the graphs and see the path taken by the robot from the start to the goal node and the quiver plots

			NOTE :::: PLEASE MAKE SURE THAT THE MATPLOTLIB GRAPHS CAN BE DISPLAYED, ESPECIALLY IF BEING RUN ON SPYDER or even any other IDE
_________________________________________________________________________________________

Code : Solves the obstacle map based on the A* Algorithm For Rigid Robot with a radius and clearance fro non- Holonomic Drive Robot

The following inputs are required by the user:

-> x_start, y_start, start_orientation
-> x_goal, y_goal
-> RPM_L, RPM_R
-> clearance

---------------------------------------------------------------------------------------------
Time for solution for the following inputs: 

approx 5 seconds for graph generation
<2 minutes for turtlebot path traversal 

Both codes takes a few minutes to solve (subject to the system specifications), and then output 
the following:

##After Reaching the goal, the turtlebot will stop moving. A list with all the traversed points will be repeatedly shown in the Terminal.


---------------------------------------------------------------------------------------------


