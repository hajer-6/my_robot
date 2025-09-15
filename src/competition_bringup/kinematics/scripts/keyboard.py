#!/usr/bin/env python3

import rospy
import pynput
from geometry_msgs.msg import Twist

class KeyboardControl:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.linear_speed = 1.2
        self.angular_speed = 1.2
        self.listener = pynput.keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.twist.linear.x = self.linear_speed
            elif key.char == 's':
                self.twist.linear.x = -self.linear_speed
            elif key.char == 'a':
                self.twist.linear.y = -self.linear_speed
            elif key.char == 'd':
                self.twist.linear.y = self.linear_speed
            elif key.char == 'q':
                self.twist.angular.z = self.angular_speed
            elif key.char == 'e':
                self.twist.angular.z = -self.angular_speed
            elif key.char == 'm':
                self.linear_speed = min(self.linear_speed + 0.05, 5.0)  
                self.angular_speed = min(self.angular_speed + 0.05, 5.0)
                rospy.loginfo(f"Speed increased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")
            elif key.char == 'n':
                self.linear_speed = max(self.linear_speed - 0.05, 0.05)  #
                self.angular_speed = max(self.angular_speed - 0.05, 0.05)
                rospy.loginfo(f"Speed decreased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in ['w', 's']:
                self.twist.linear.x = 0
            elif key.char in ['a', 'd']:
                self.twist.linear.y = 0
            elif key.char in ['q', 'e']:
                self.twist.angular.z = 0
        except AttributeError:
            pass

    def run(self):
        self.pub.publish(self.twist)

if __name__ == "__main__":
    rospy.init_node("keyboard_control_node")
    kb = KeyboardControl()
    rate = rospy.Rate(20) 

    rospy.loginfo("Keyboard control started. Use WASD to move, Q/E to rotate, M/N to change speed.")

    while not rospy.is_shutdown():
        kb.run()
        rate.sleep()
