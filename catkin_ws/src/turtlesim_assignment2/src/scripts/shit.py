#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Publisher which will publish to the topic '/turtle1/cmd_vel'.
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
# when a message of type Pose is received.


def update_pose(data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    pose = data
    pose.x = round(pose.x, 4)
    pose.y = round(pose.y, 4)
    pose.theta = round(pose.theta, 4)


def move2goal(X, Y, tolerance):

    """Moves the turtle to the goal."""
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    goal_pose = Pose()
    pose = Pose()
    goal_pose.x = X
    goal_pose.y = Y
    
    distance = math.sqrt(math.pow((goal_pose.x - pose.x), 2) + math.pow((goal_pose.y - pose.y), 2))

    # Please, insert a number slightly greater than 0 (e.g. 0.01).
    distance_tolerance = tolerance

    vel_msg = Twist()

    while distance >= distance_tolerance:

        # Linear velocity in the x-axis.
        vel_msg.linear.x = 1.5 * distance
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 6 * (math.atan2(goal_pose.y - pose.y, goal_pose.x - pose.x) - pose.theta)


        # Publishing our vel_msg
        velocity_publisher.publish(vel_msg)

    # Stopping our robot after the movement is over.
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    # If we press control + C, the node will stop.
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('turtlsim_assignment2', anonymous=True)
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, update_pose)
        move2goal(5, 5, 0.001)
        move2goal(8, 5, 0.001)
        move2goal(8, 8, 0.001)
        move2goal(5, 8, 0.001)
        move2goal(5, 5, 0.001)
    except rospy.ROSInterruptException: pass