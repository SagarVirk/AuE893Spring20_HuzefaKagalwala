#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import time
import numpy

kps = 1.2
kds = 0.00065
kis = 0.05
uk1s = 0.0
er_1s = 0.0
er_2s = 0.0


def callback(points):
    frontleft = points.ranges[0:79]
    frontright = points.ranges[280:359]
    left = []
    right = []
    global kps, kds, kis, uk1s, er_1s, er_2s
    for i in frontleft:
        if 0 < i < 10:
            left.append(i)
    for j in frontright:
        if 0 < j < 10:
            right.append(j)
    errors = numpy.mean(right) - numpy.mean(left)
    # velocity.angular.z = -kps*errors
    # velocity.linear.x = 0.8

    k_1s = kps + kis + kds
    k_2s = -kps - (2.0 * kds)
    k_3s = kds

    uks = uk1s + (k_1s * errors) + (k_2s * er_1s) + (k_3s * er_2s)
    uks = uks
    uk1s = uks
    er_2s = er_1s
    er_1s = errors

    velocity.linear.x = 0.1

    if errors == 0:
        velocity.angular.z = 0.0
    else:
        velocity.angular.z = -uks
    return velocity


rospy.init_node('wall_follower', anonymous=True)
velocity = Twist()
publish = rospy.Publisher('cmd_vel', Twist, queue_size=10)
subscribe = rospy.Subscriber('scan', LaserScan, callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    publish.publish(velocity)
    rate.sleep()
