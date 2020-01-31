ASSIGNMENT 2 README FILE
SUBMITTED BY: Huzefa Shabbir Hussain Kagalwala (C48290423)

circle.py:
This script moves the turtlebot in a circle with a constant angular veocity of 10 rad/s.
We initialize a velocity message whose linear X and angular Z we set to the desired angular velocity we want. By constantly publishing this Twist message, the turtlebot moves constantly at that speed.
To launch this file use the following command: roslaunch circle_py.launch

square_openloop.py:
This script moves the turtlebot in a square of 2x2 dimensions with a linear as well as angular speed of 0.2 units.
We initialize a velocity message whose linear X and angular Z we set to the desired angular velocity we want. By publishing this Twist message, till the desired distance and angles are reached, the turtlebot moves in the square of the desired shape.
To launch this file use the following command: roslaunch square_openloop.launch

square_closedloop.py:
This script moves the turtlebot towards the coordinates mentioned in the Assignment file.
We initialize a velocity message whose linear X and angular Z we set to the desired angular velocity we want. We also subscribe to the pose of the turtlebot so that we can manipulate it in the direction we want. There are functions defined which make use of this pose subscription nannd manipulate  it such that the turtlebot reaches the coordinates one after the other.
To launch this file use the following command: roslaunch square_closedloop.launch

NOTE: The screenshots of the trajectories are provided in the "Trajectories" folder provided
