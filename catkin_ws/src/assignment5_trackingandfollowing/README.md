### ASSIGNMENT 5 TRACKING & FOLLOWING

Submitted on 04/02/2020, by Team 8:

1. Mohammad Anas Imam Khan (C17566828), responsible for line following implementation in Gazebo and on the TurtleBot3 and the subsequent launch files
2. Prajval Vaskar (C20664702), responsible for the April Tag tracking script and implementation on the TurtleBot3 and launch file
3. Sagar Virk (C20531221), responsible for the April Tag tracking script and implementation on the TurtleBot3 and launch file
4. Huzefa Shabbir Hussain Kagalwala (C48290423), responsible for ine following implementation in Gazebo and on the TurtleBot3 and the subsequent launch files, assignment submission
5. Adam Wagner (C10835588), responsible for ine following implementation in Gazebo and on the TurtleBot3 and the subsequent launch files

Even though these were the individual contributions, knowledge transfer has ensured that every team member is conversant with all the aspects of the assignment.

## Camera Calibration

1. Before starting the assignment the Raspberry Camera interface was setup and relevant packages were installed by following the instructions in the [TurtleBot3 manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/) 
2. The camera calibration was performed by running the following commands:
	- `roscore`
	- ssh into the TurtleBot and run the command `roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true`
	- In a new terminal run the command: `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/raspicam_node/image camera:=/raspicam_node`
**NOTE:** 
- The packages for the camera calibration and the instructions to run the package were obtained from [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- The checkerboard used was a 8x6 board printed on an A4 sheet, with a square size of 0.0254 m.
	

## Part 1

Part 1 required us to implement line following on the TurtleBot in Gazebo and on the physical robot.

For the Gazebo part, we have used a TurtleBot3 Waffle instead of a Burger as the Waffle has a camera which is essential for the simulation. Also, we installed OpenCV2 on our systems before proceeding with the assignment.

1. The CV Bridge and vision_opencv packages were copied in the `src` folder of our workspaces as provided.
2. The .bashrc file was modified so that it selected the world and its elements which were created for this assignment correctly.
3. The `follow_line_step_hsv.py` script was used as the template. In this script, we implemented a controller to follow the centroid formed on the line of whichever color we want it to track.
4. The image crop was also tuned so that the robots field of view is sufficient for the tracking to take place.
5. Towards this end, a launch file was created called **turtlebot3_follow_line_step_hsv.launch** which launches Gazebo and the line following script 
6. To use the launch file use the command: `roslaunch assignment5_trackingandfollowing turtlebot3_follow_line_step_hsv.launch`

For the physical implementation of the robot run the following commands:
- Run `roscore` in a terminal.
- ssh into the TurtleBot and run `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
- In another terminal, ssh into the TurtleBot and run `roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch`
- In another terminal run `rosrun image_transport republish compressed in:=raspicam_node/image raw out:=raspicam_node/image_raw`
- Open another terminal and run the launch file using `roslaunch assignment5_trackingandfollowing turtlebot3_follow_line_step_hsv_bot.launch`
- The video showing the implementation are provided in the **videos** folder.

The code was implemented on a blue tape. To achieve this, the BGR color range was modified accordingly. Also, changes to the image crop to adjust the FOV of the robot and controller tune were made.


## Part 2

Part 2 required us to make the TurtleBot3 track AprilTags

- Initially, the AprilTag packages were installed from [here](https://github.com/AprilRobotics/apriltag_ros).
- The **Tag36h11** family of April Tags were printed on an A4 sheet of paper.

To run the this part of the assignment, run the following commands:
1. Run `roscore` in a terminal.
2. ssh into the TurtleBot and run `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
3. In another terminal, ssh into the TurtleBot and run `roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch`
4. In another terminal run `rosrun image_transport republish compressed in:=raspicam_node/image raw out:=raspicam_node/image_raw`
5. Open another terminal and run the launch file using `roslaunch assignment5_trackingandfollowing apriltag_detection.launch`

The **settings.yaml** file contains the information that the 36h11 family of tags are being used.
The **tags.yaml** file contains the ID of the tag which we want to track. We can use a single or multiple tags.
These files have been provided in `../launch` folder of the assignment.

The code implementation is explained as follows:
- The April Tag package publishes the X, Y and Z positions of the tag relative to the robot in the following message **AprilTagDetectionArray** in the **/tag_detections** topic.
- These relative positions are used to implement a controller for forward and rotary motion such that the robot tracks the April Tag and comes to a halt at a distance of 0.15 metres from the tag.
- The videos showing the implementation are provided in the **videos** folder.



