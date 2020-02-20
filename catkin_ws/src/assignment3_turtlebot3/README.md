### ASSIGNMENT 3 TURTLEBOT 3

Submitted on 02/20/2020, by Team 8:

1. Mohammad Anas Imam Khan (C17566828), responsible for obstacle avoidance script and its launch file
2. Prajval Vaskar (C20664702), responsible for the world file with the wall
3. Sagar Virk (C20531221), responsible for the circle and square scripts and its launch file
4. Huzefa Shabbir Hussain Kagalwala (C48290423), responsible for obstacle avoidance script and its launch file
5. Adam Wagner (C10835588), responsible for the circle and square scripts and launch its file

Even though these were the individual contributions, knowledge transfer has ensured that every team member is conversant with all the aspects of te assignment.

## Part 1

Part 1 required us to recreate the previous TurtleSim simulations in Gazebo using TurtleBot3 burger.

Towards this end, a launch file called **move.launch** was created which launches ROS, Gazebo and the simulation script we want to run based on the argument "code".

The initial lines of the software launches an empty world in Gazebo with the TurtleBot spawning at (0,0,0) in that world.
The second part of the script looks for the argument "code" and based on the input received in the command line, launches the node to execute the scripts which will make the TurtleBot move in a circle or square.

To launch the script which makes the Turtlebot3 Burger move in a circle in Gazebo, type the following command: `roslaunch assignment3_turtlebot3 move.launch code:=circle`.
To launch the script which makes the Turtlebot3 Burger move in a square in Gazebo, type the following command: `roslaunch assignment3_turtlebot3 move.launch code:=square`.

## Part 2

Part 2 required us to make a world file with a wall inside which would act as an obstacle. The TurtleBott should be able to detect the wall and stop or move in a straight line otherwise.

An empty world was launched and a wall was added using the Building Editor in Gazebo. This world file was saved as **turtlebot3_wall.world** in the folder where all the worlds are called from (`.../turtlebot3_simulations/turtlebot3_gazebo/worlds/`).

Similarly, a launch file was created which launches ROS, Gazebo and the obstacle avoidance script called **turtlebot3_wall.launch**.
The initial lines of the software launches the world with the wall in Gazebo with the TurtleBot spawning at (0,0,0) in that world.
The second part of the script launches the node to execute the script which will make the TurtleBot move in a straight line until it detects the wall (The TurtleBot will stop when it is 0.8m from the wall).

The approach to achieve the obstacle detection is as follows:
1. The FOv was mapped and restricted to 80 degrees (0 to 39 and 359 to 319).
2. Rather than finding the middle point which works only for obstacles which are dead ahead of the bot when it is moving straight (no yaw) and/or the obstacle has no curvature, we find the minimum distance to the obstacle in that FOV. This helps us to incorporate any curvature in the obstacle or pose of the robot.

To run this correctly, you will need to clone even the turtlebot3 package folders or place the **turtlebot3_wall.world** file provided in the "scripts folder" in the folder from where you call your gazebo worlds.
To launch this script, run the following command: `roslaunch assignment3_turtlebot3 turtlebot3_wall.launch`.

