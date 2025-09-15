#!/usr/bin/env python3

from imu_message.imu_bno055_api import BoschIMU
import rospy
from imu_message.SensorIMU import * 
import time
from sensor_msgs.msg import Imu

def run(imu_sensor):
    imu_sensor.publish_imu()
    yaw_radians, yaw_degrees = imu_sensor.get_yaw()
    pitch_radians, pitch_degrees = imu_sensor.get_pitch()
    roll_radians, roll_degrees = imu_sensor.get_roll()
    rospy.loginfo(f"Yaw: {yaw_radians:.2f} rad, {yaw_degrees}  degrees")
    rospy.loginfo(f"Pitch: {pitch_radians:.2f} rad, {pitch_degrees} degrees")
    rospy.loginfo(f"Roll: {roll_radians:.2f} rad,  {roll_degrees} degrees")

try:
    imu=BoschIMU(port="/dev/ttyUSB0")
except:
    imu=BoschIMU(port="/dev/ttyUSB1")

imu_msg=Imu()
imu_sensor = ROSIMU(imu,imu_msg)

def main():
    """Main function to run the IMU listener."""
    rospy.init_node('imu_listener_node', anonymous=False)
    
    
    rate = rospy.Rate(75) 
    while not rospy.is_shutdown():
        run(imu_sensor)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
