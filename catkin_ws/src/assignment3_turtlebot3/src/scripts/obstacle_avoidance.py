#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np

def callback():
    calc_dist = []
    calc_dist2 = []
    dist = 9999
    vel = Twist()
    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    while dist>=0.8:
        scan_msg=rospy.wait_for_message('/scan',LaserScan)
        for i in range(10):
            calc_dist.append(scan_msg.ranges[i+1])
            calc_dist2.append(scan_msg.ranges[i+(len(scan_msg.ranges)-11)])
        final = calc_dist+calc_dist2
        Range = len(final)
        if final[Range/2]<=final[(Range/2)-1]:
            dist=final[Range/2]
        else:
            dist=final[(Range/2)-1]
        vel.linear.x = 0.5
        vel.linear.y = 0    
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0 
        vel.angular.z = 0.0
        vel_pub.publish(vel)
    vel.linear.x = 0.0
    vel_pub.publish(vel)
    rospy.loginfo("Obstacle too close, turtlebot stopped")

if __name__=='__main__':    
    rospy.init_node('turtlebot3_world',anonymous=True)    
    callback()
    rospy.spin()

