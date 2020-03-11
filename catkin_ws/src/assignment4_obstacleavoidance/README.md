### ASSIGNMENT 4 TURTLEBOT 3

Submitted on 03/10/2020, by Team 8:

1. Mohammad Anas Imam Khan (C17566828), responsible for the wall following script and its launch file
2. Prajval Vaskar (C20664702), responsible for the wander script and its launch file
3. Sagar Virk (C20531221), responsible for the wall following script and its launch file
4. Huzefa Shabbir Hussain Kagalwala (C48290423), responsible for the wander script and its launch file, assignment submission
5. Adam Wagner (C10835588), responsible for implementation of wander.py script on the real Turtlebot

Even though these were the individual contributions, knowledge transfer has ensured that every team member is conversant with all the aspects of the assignment.

## Part 1

Part 1 required us to implement a PID controller to make the bot follow a path between the walls.

Towards this end, a launch file called **turtlebot3_wallfollowing.launch** was created which launches ROS, Gazebo and the simulation script we want to run.

The initial lines of the launch code launches the wall following world in Gazebo with the TurtleBot.
The second part of the launch script launches the node to execute the scripts which will make the TurtleBot follow a path between the walls.

To achieve the objective, the following approach was used:
1. Just used the front FOV but increased it to have high fidelty view. (FOV was 160 degrees) 
2. Direct PID implementation was very difficult, so it was decided to implement digitized PID control
3. The control parameters were set as such that the bot follows the walls, keeping equal distance from them.
4. The PID control was only implemented to govern the steering of the bot and linear velocities were kept constant.

To launch the script which makes the Turtlebot3 Burger follow the walls, type the following command: `roslaunch assignment4_turtlebot3 turtlebot3_wallfollowing.launch`.

## Part 2

Part 2 required us to make the TurtleBot3 wander about infinitely while navigating an environment filled with obstacles.

A launch file was created which launches ROS, Gazebo and the wander.py script, called **turtlebot3_obstacleavoidance.launch**.
The initial lines of the code launches the world in Gazebo with the TurtleBot spawned in it.
The second part of the script launches the node to execute the script which will make the TurtleBot wander about while avoiding the obstacles in the world.

The approach to achieve the obstacle detection is as follows:
1. The FOV of the robot was divided into 3 sectors:
	- Front sector, having a sector angle of 40 degrees.
	- Right sector, having a sector angle of 70 degrees.
	- Left sector, having a sector angle of 70 degrees.
2. Rather than finding the middle point which works only for obstacles which are dead ahead of the bot when it is moving straight (no yaw) and/or the obstacle has no curvature, we find the minimum distance to the obstacle in that FOV. This helps us to incorporate any curvature in the obstacle or pose of the robot.
3. The robot moves in a straight line with constant linear velocity, till it doesn't see any obstacle in it's front (till a threshold of 0.5 units). When it does so, the robot checks whether there is more space in the left or the right. Wherever, there is more space, the robot turns. This is how it keeps on moving around aimlessly, while avoiding obstacles.

To launch this script, run the following command: `roslaunch assignment4_obstacleavoidance turtlebot3_obstacleavoidance.launch`.

**NOTE:** 
- We have given the requisite world files, in a seperate folder in the assignment. Please paste the world files in the appropriate location in your workspace. (`.../turtlebot3_simulations/turtlebot3_gazebo/worlds/`)
- There are two scripts for "wander". The real world script (launched from the turtlebot3_obstacleavoidance _turtlebot.launch file), has the parameters tuned for running the bot in the real world).

