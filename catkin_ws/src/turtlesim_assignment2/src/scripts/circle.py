#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

def rotate():
    # Starts a new node
    rospy.init_node('turtlesim_assignment2', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    speed = 5

    #Since we are moving just in x-axis
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = speed

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        rotate()
    except rospy.ROSInterruptException: pass