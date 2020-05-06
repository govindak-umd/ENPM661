Steps to run the workspace called toycar_ws  :


copy the workspace (toycar_ws) and launch it:

		roslaunch toycar gazebo.launch

The rviz launch has been terminated because of errors"

Hence, to launch RVIZ, run 

		roslaunch toycar display.launch

The python script called controller.py has been integrated to gazebo.launch itself. 

It alligns the wheels ahead. And sets the vehicles velocity to 0 m/s.
Within the controller.py, the wheels can be adjusted based on the position and has been given
a maximum angle of 28 degrees.

The vehicle will remain stationary, until the vel variable is changed.

The map has been saved using SLAM as well, and a image of that has been included as well.
The tf frames have been included under the src folder as well and can be seen by navigating to the folder and typing 

		evince frames.pdf
