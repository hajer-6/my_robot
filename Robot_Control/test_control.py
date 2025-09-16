#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
import math
from tf.transformations import euler_from_quaternion

forward_distance = right_distance = left_distance = yaw = 0
angular_speed = 0.3

def ultrasonicf(msg):
    global forward_distance

    forward_distance = msg.range * 100
    rospy.loginfo("Forward: %d ",forward_distance)
def ultrasonicr(msg):
    global right_distance

    right_distance = msg.range * 100
    rospy.loginfo("right: %d ",right_distance)
def ultrasonicl(msg):
    global left_distance

    left_distance = msg.range * 100
    rospy.loginfo("left: %d ",left_distance)


def yaw_callback(msg):
    global yaw
    # print("received imu data")
    q = msg.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    # print("Yaw (rotation about Z):", yaw)

def listener():
    rospy.init_node('robot')
    rospy.Subscriber('ultrasonic/front', Range, ultrasonicf)
    rospy.Subscriber('ultrasonic/right', Range, ultrasonicr)
    rospy.Subscriber('ultrasonic/left', Range, ultrasonicl)
    rospy.Subscriber('/imu', Imu, yaw_callback)

def normalize_angle(angle):
    """Wrap angle to [-180, 180] degrees."""
    return math.atan2(math.sin(angle), math.cos(angle))


def movement():
    cmd = Twist()

    # safety check
    if forward_distance == 0 or left_distance == 0 or right_distance == 0:
        cmd.linear.x = 0
        cmd.angular.z = 0
        pub.publish(cmd)
        return

    # If obstacle ahead
    if forward_distance < 40:
        turn_angle(math.pi/2, cmd) # turn left for now, untill we have cv readings
    else:
        cmd.linear.x = min(0.1, forward_distance * 0.01)
        cmd.angular.z = 0
        pub.publish(cmd)


def turn_angle(rotation_angle, cmd):
    """Turn the robot by a relative angle (radians)."""
    global target_yaw
    start_yaw = yaw
    target_yaw = normalize_angle(start_yaw + rotation_angle)
    rate = rospy.Rate(10)

    error = normalize_angle(target_yaw - yaw)
    while not rospy.is_shutdown() and abs(error) > 0.1:
        cmd.angular.z = math.copysign(angular_speed, error)
        cmd.linear.x = 0
        pub.publish(cmd)
        rate.sleep()
        error = normalize_angle(target_yaw - yaw)  # update inside loop



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
