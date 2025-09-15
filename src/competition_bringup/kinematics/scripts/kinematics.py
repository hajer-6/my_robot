#!/usr/bin/env python3
import rospy
import numpy as np
from mecanum_kinematics import kinematicModel
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# Model specifications
wheel_radius = 0.04  
lx = 0.13 
ly = 0.15  

FREQ = 50
SPEEDS_TOPIC_NAME = "/setpoints"

kinematic = kinematicModel(wheel_radius, lx, ly)

latest_twist = Twist()

def cmd_vel_callback(msg):
    global latest_twist
    latest_twist = msg 

if __name__ == '__main__':
    rospy.init_node("movement_node")
    rospy.loginfo("Starting Movement Node")

    pub_speeds = rospy.Publisher(SPEEDS_TOPIC_NAME, Float32MultiArray, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    msg = Float32MultiArray()
    rate = rospy.Rate(FREQ)

    while not rospy.is_shutdown():
        speeds = kinematic.mecanum_4_vel_forward(
            latest_twist.linear.x,
            - latest_twist.linear.y,
            latest_twist.angular.z
        )

        msg.data = speeds
        pub_speeds.publish(msg) 
        rate.sleep()
