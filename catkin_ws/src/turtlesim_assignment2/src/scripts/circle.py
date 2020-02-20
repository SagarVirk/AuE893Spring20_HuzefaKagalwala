#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

def rotate(speed, angle, Clockwise):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    #Converting from angles to radians
    relative_angle = angle*math.pi/180

    #We wont use the following velocity components
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if (Clockwise == True):
        vel_msg.angular.z = -abs(speed)
        vel_msg.linear.x = speed
    else:
        vel_msg.angular.z = abs(speed)
        vel_msg.linear.x = speed
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0

    while (current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        rospy.init_node('turtlesim_assignment2', anonymous=True)
        rotate(2,380,True)
    except rospy.ROSInterruptException: pass