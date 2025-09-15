#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray
import math

forward_distance = right_distance = left_distance = yaw = turned = moved = target_yaw = 0
max_velocity = 0.2
kp = 0.3
rotaton_tolerance = 2


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
    global target_yaw, moved, turned
    cmd = Twist()

    # safety check
    if forward_distance == 0 or left_distance == 0 or right_distance == 0:
        cmd.linear.x = 0
        cmd.angular.z = 0
        pub.publish(cmd)
        return

    # If obstacle ahead
    if forward_distance < 5 and moved:
        turned = 1
        turn_angle(90, cmd) # turn left for now, untill we have cv readings

    else:
        moved = 1
        error = normalize_angle(target_yaw - yaw)
        if abs(error) > rotaton_tolerance and turned:
            cmd.linear.x = min(max_velocity, (forward_distance - 5) * 0.01)
            error_rad = math.radians(error)
            p_rotation = error_rad * kp # the p in pid
            cmd.angular.z = p_rotation
        else:
            cmd.linear.x = min(max_velocity, (forward_distance - 5) * 0.01)
            cmd.angular.z = 0
            
        pub.publish(cmd)



def turn_angle(rotation_angle, cmd):
    # Turn the robot by a relative angle
    global target_yaw
    start_yaw = yaw
    target_yaw = normalize_angle(start_yaw + rotation_angle)
    turn_rate = rospy.Rate(30)

    error = normalize_angle(target_yaw - yaw)
    while not rospy.is_shutdown() and abs(error) > rotaton_tolerance:
        error_rad = math.radians(error)
        p_rotation = error_rad * kp # the p in pid
        cmd.angular.z = p_rotation
        cmd.linear.x = 0
        pub.publish(cmd)
        turn_rate.sleep()
        error = normalize_angle(target_yaw - yaw)  # update error inside loop

def snap_angle(angle_deg):
    # Snap an angle in degrees to the closest of [0, 90, -90, 180]
    targets = [0, 90, -90, 180]
    return min(targets, key=lambda x: abs(angle_deg - x))

def main():
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    listener()
    main_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        movement()
        main_rate.sleep()
        

if __name__ == "__main__":
    main()

# roslaunch turtlebot3_gazebo turtlebot3_house.launch
# roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
# :~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf
