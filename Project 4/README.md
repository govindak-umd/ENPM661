README File
_________________________________________________________________________________________

				Project 4 | ENPM 661 | Planning for Autonomous Robots |
					Baxter Arm Pick and Place using ROS, Python, OMPL, Moveit

						SUBMISSION : April 17, 2020

--------------------------------------------------%%-------------------------------------------
	
	GITHUB: 

	https://github.com/govindak-umd/ENPM661/tree/master/Project%204



--------------

	YOUTUBE:

	https://youtu.be/jf0WXoY2Kxw
	https://youtu.be/VjC5dWsRl9M
	https://youtu.be/gSKFNhe4hHQ

--------------------------------------------------%%-------------------------------------------

Please run the following commands in order. Wherever (-----------) is drawn out, please open a new terminal.

To start ROS
----------
		roscore 
		
To launch the Baxter Simulator
----------
		cd ~/ros_ws
		catkin_make
		source ./devel/setup.bash
		./baxter.sh sim
		roslaunch baxter_gazebo baxter_world.launch

To run the joint_trajectory_action_server fro RViz
----------
		cd ~/ros_ws
		catkin_make
		source devel/setup.bash
		rosrun baxter_tools enable_robot.py -e
		rosrun baxter_interface joint_trajectory_action_server.py
To Run RViz
----------
		cd ~/ros_ws
		catkin_make
		source devel/setup.bash
		roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true
To get the coordinate values of the end points of the gripper (here, left limb)
----------
		cd ~/ros_ws
		catkin_make
		source devel/setup.bash
		rostopic echo /robot/limb/left/endpoint_state 
To run the Python code, titled ik_pick_and_place_demo.py
----------
		cd ~/ros_ws
		catkin_make
		source devel/setup.bash
		rosrun baxter_sim_examples ik_pick_and_place_demo.py
To get the joint angles of the end effector open a new terminal and type : 
----------
		cd ~/ros_ws
		catkin_make
		source devel/setup.bash
		rosrun baxter_examples ik_service_client.py -l left
To get the coordinate value fo any point in space on RViz, use the following echo command
----------
		rostopic echo /clicked_point


_________________________________________________________________________________________
Authors: 

Govind Ajith Kumar
UID : 116699488

Masters Robotics 
University of Maryland
College Park
Maryland
20740 USA

Rajeshwar NS
UID : 116921237

Masters Robotics
University of Maryland
College Park
Maryland
20740 USA
_________________________________________________________________________________________
Language: Python 3.7.x

USES ROS, OMPL, MOVEIT (Refer to PPT for directions to install)
THIS PROGRAMME DOES NOT TAKE ANY USER INPUTS
_________________________________________________________________________________________
Libraries implemented and required to be installed for the code to execute:



	ROS and Python Imports

	import argparse
	import struct
	import sys
	import copy
	import time
	import rospy
	import rospkg

	from gazebo_msgs.srv import (
	    SpawnModel,
	    DeleteModel,
	)
	from geometry_msgs.msg import (
	    PoseStamped,
	    Pose,
	    Point,
	    Quaternion,
	)
	from std_msgs.msg import (
	    Header,
	    Empty,
	)

	from baxter_core_msgs.srv import (
	    SolvePositionIK,
	    SolvePositionIKRequest,
	)

	import baxter_interface

_________________________________________________________________________________________


---------------------------------------------------------------------------------------------



