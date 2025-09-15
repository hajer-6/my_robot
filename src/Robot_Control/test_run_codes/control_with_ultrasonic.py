#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray
import math

forward_distance = right_distance = left_distance = yaw = moved = 0
max_velocity = 0.2
kp = 0.3


def ultrasonics_callback(msg):
    global forward_distance, right_distance, left_distance
    # rospy.loginfo('got ultrasonic data')
    forward_distance = msg.data[0]
    right_distance = msg.data[1]
    left_distance = msg.data[2]
    rospy.loginfo("Forward: %f, Right: %f, Left: %f",
    forward_distance, right_distance, left_distance)

def yaw_callback(msg):
    global yaw
    rospy.loginfo('got yaw data')
    yaw = msg.data
    rospy.loginfo("Yaw: %f", yaw)

def listener():
    rospy.init_node('robot')
    rospy.Subscriber('ultrasonics', Int16MultiArray, ultrasonics_callback)
    rospy.Subscriber('yaw', Int16, yaw_callback)



def normalize_angle(angle):
    # Wrap angle to [-180, 180] degrees
    return ((angle + 180) % 360) - 180


def movement():
    global moved
    cmd = Twist()

    # safety check
    if forward_distance == 0 or left_distance == 0 or right_distance == 0:
        cmd.linear.x = 0
        cmd.angular.z = 0
        pub.publish(cmd)
        return

    # If obstacle ahead
    if forward_distance < 5 and moved:
        turn("left", cmd) # turn left for now, untill we have cv readings
    else:
        moved = 1
        cmd.linear.x = min(max_velocity, (forward_distance - 5) * 0.01)
        cmd.angular.z = 0
        pub.publish(cmd)


def turn(direction, cmd):
    target_distance1 = forward_distance + 8 # distance + difference in length and width

    if direction == "right":
        target_distance2 = right_distance - 8 # distance + difference in length and width
        error1 = left_distance - target_distance1
        error2 = forward_distance - target_distance2

    else:
        target_distance2 = left_distance - 8 # distance + difference in length and width
        error1 = target_distance1 - right_distance
        error2 = target_distance2 - forward_distance
    
    error_sum = error1 + error2
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and abs(error_sum) > 1:
        error_mapped = error_to_rad(error_sum)
        p_rotation = error_mapped * kp # the p in pid
        cmd.angular.z = p_rotation
        cmd.linear.x = 0
        pub.publish(cmd)
        rate.sleep()

        # update error inside loop
        if direction == "right": 
            error1 = left_distance - target_distance1
            error2 = forward_distance - target_distance2

        else:
            error1 = target_distance1 - right_distance
            error2 = target_distance2 - forward_distance
        error_sum = error1 + error2

def error_to_rad(error):
    max_error = 10
    max_turn_speed = 0.5
    error_clamped = max(-max_error, min(max_error, error))  # clamp
    error_mapped = (error_clamped / max_error) * max_turn_speed
    return error_mapped


def main():
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    listener()

    while not rospy.is_shutdown():
        movement()
        

if __name__ == "__main__":
    main()

# roslaunch turtlebot3_gazebo turtlebot3_house.launch
# roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
# :~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf