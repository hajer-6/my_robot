#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray

forward_distance = right_distance = left_distance = yaw =  None
vision_input = ""
target_yaw = 0
cmd = Twist()


kp_rotation = 0.5
kp_adjust = 1.5
kp_linear = 0


def ultrasonics_callback(msg):
    global forward_distance, right_distance, left_distance
    rospy.loginfo('got ultrasonic data')

    forward_distance = msg.data[0]
    right_distance = msg.data[1]
    left_distance = msg.data[2]
    rospy.loginfo("Forward: %d, Right: %d, Left: %d",
                      forward_distance, right_distance, left_distance)

def yaw_callback(msg):
    global yaw
    rospy.loginfo('got yaw data')
    yaw = msg.data
    rospy.loginfo("Yaw: %d", yaw)

def listener():
    rospy.init_node('robot')
    rospy.Subscriber('ultrasonics', Int16MultiArray, ultrasonics_callback)
    rospy.Subscriber('yaw', Int16, yaw_callback)
    rospy.spin()




def new_yaw(target_yaw,vision_input):
    if vision_input == "left":
        target_yaw += 90
    elif vision_input == "right":
        target_yaw -= 90
    if left_distance > 11:
        target_yaw += 90
    elif right_distance > 11 :
        target_yaw -= 90
    else:
        target_yaw += 180 #just in case we go down a dead end
    if target_yaw > 180:
        target_yaw -= 360
    elif target_yaw <-180:
        target_yaw += 360

    return target_yaw


def turn(target_yaw):
    error = target_yaw - yaw
    cmd.angular.z = error*kp_rotation



def yaw_adjust(target_yaw):
    error = left_distance - right_distance
    target_yaw += error*kp_adjust
    return target_yaw








def movement():


    if forward_distance == None:
        cmd.linear.x = 0
        cmd.angular.z = 0
    elif forward_distance > 5: #moving state

        turn(target_yaw)
        velocity = (forward_distance -5)*kp_linear
        # velocity = max(min(velocity, 5), 5)  # clamp speed to 5
        cmd.linear.x = velocity
    else: #turning state
        target_yaw = new_yaw(target_yaw, vision_input)
        cmd.linear.x = 0
        turn(target_yaw)

    pub.publish(cmd)


if __name__ == "__main__":
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    listener()
    while not rospy.is_shutdown():
       movement()