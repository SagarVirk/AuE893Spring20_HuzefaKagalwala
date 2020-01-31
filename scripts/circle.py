#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
pi = 3.1415926535897

def rotate():
    # Starts a new node
    rospy.init_node('turtlesim_assignment2', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = 30 #Speed input is in degrees/sec
    angle = 360 #Angle in degrees
    clockwise = True
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    #Checking if the movement is clockwise or anti-clockwise
    if(clockwise == True):
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    #Since we are moving just in x-axis
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        #Loop to move the turtle in an specified distance
        while(current_angle < relative_angle):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_angle = angular_speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.angular.z = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        rotate()
    except rospy.ROSInterruptException: pass