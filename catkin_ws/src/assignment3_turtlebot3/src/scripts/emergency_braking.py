#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import time
import numpy as np

def callback(points):
    leftdata = points.ranges[0:39] # This creates the half FOV in the positive direction of 40 deg
    rightdata = points.ranges[319:359] # This creates the half FOV in the negative direction of 40 deg
    usefuldata = leftdata + rightdata # This completes the full 80deg FOV cone
    if min(usefuldata)<= 0.8: # Condtition for the bot to stop at 0.8m from wall
        velocity.linear.x = 0
    else:
        velocity.linear.x = 0.5
    return velocity


rospy.init_node('turtlebot3_world', anonymous=True)
velocity = Twist()
publish = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
subscribe = rospy.Subscriber('scan', LaserScan, callback)
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    publish.publish(velocity)
    rate.sleep()
