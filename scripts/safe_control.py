#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray
import math

forward_distance = right_distance = left_distance = yaw = moved = 0
max_velocity = 0.8
kp_linear = 1.5

kp_angular = 0.7
ki_angular = 0
kd_angular = 0

rotation_tolerance = 4

last_error = 0
integral = 0
dt = 0.1

def pid_control(error):
    global last_error, dt, integral

    # Proportional
    p = kp_angular * error

    # Integral
    integral += error * dt
    i = ki_angular * integral

    # Derivative
    derivative = (error - last_error) / dt if dt > 0 else 0
    d = kd_angular * derivative

    # Save error for next cycle
    last_error = error

    # PID output
    return p + i + d





def ultrasonics_callback(msg):
    global forward_distance, right_distance, left_distance
    forward_distance = msg.data[0]
    right_distance = msg.data[1]
    left_distance = msg.data[2]
    rospy.loginfo("Forward: %f, Right: %f, Left: %f",
    forward_distance, right_distance, left_distance)

def yaw_callback(msg):
    global yaw
    yaw = msg.data
    rospy.loginfo("Yaw: %f", yaw)

def listener():
    rospy.init_node('robot')
    rospy.Subscriber('ultrasonics', Int16MultiArray, ultrasonics_callback)
    rospy.Subscriber('yaw', Int16, yaw_callback)



def normalize_angle(angle):
    # Wrap angle to [-180, 180] degrees
    return ((angle + 180) % 360) - 180

def normalize_yaw(angle):
    # Keep yaw in [0, 360) degrees
    return angle % 360


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

    if forward_distance < 8 and moved:
        turn_angle(90, cmd) # turn right

    else:
        moved = 1
        cmd.linear.x = min(max_velocity, (forward_distance - 8) * kp_linear)
        cmd.angular.z = 0
        pub.publish(cmd)
    


def turn_angle(rotation_angle, cmd):
    # Turn the robot by a relative angle
    global target_yaw, last_error, integral
    start_yaw = yaw
    target_yaw = normalize_yaw(start_yaw + rotation_angle) # try to snap angle here
    turn_rate = rospy.Rate(30) # reduce the turn rate if the imu is very noisy

    last_error = 0
    integral = 0

    error = normalize_angle(target_yaw - yaw)
    while not rospy.is_shutdown() and abs(error) > rotation_tolerance: # increase tolerance if it misses it
        control = pid_control(error)
        control_rad = math.radians(control)
        cmd.angular.z = control_rad
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


# after rotating = check side ultrasonic reading if like old forward one, move slowly with z correction until it is

# check seconds for a full turn, and hard code it using ros.sleep()
