#!/usr/bin/env python3
## @file
#  @brief ROS IMU node for publishing sensor data from BNO055 IMU sensor
#
#  This file implements classes to initialize, configure, and publish IMU data
#  This file have two class one with ros and other without ros
import time
import math
import rospy
from imu_message.imu_bno055_api import *  
from sensor_msgs.msg import Imu
## @class SensorIMU
#  @brief Manages IMU sensor configurations and data processing.
#
#  Initializes, configures, and updates IMU data, including orientation and acceleration.
class SensorIMU:
    ## Constructor for initializing SensorIMU instance
    #  @param imu Reference to the BoschIMU device object
    #  @param imu_msg Reference to the IMU device object
    #  @param frame_id Reference frame ID for ROS messages, default is 'imu_link'
    #  @param operation_mode Operational mode of the IMU ('IMU', 'COMPASS', 'NDOF_FMC_OFF', 'NDOF')
    #  @param oscillator Oscillator type, default is 'INTERNAL'
    def __init__(self, imu , imu_msg, frame_id='imu_link', operation_mode='IMU', oscillator='INTERNAL'):
        self.frame_id = frame_id
        self.operation_mode_str = operation_mode
        self.oscillator_str = oscillator
        self.frequency = 20

        operation_modes = {'IMU': IMU, 'COMPASS': COMPASS, 'NDOF_FMC_OFF': NDOF_FMC_OFF, 'NDOF': NDOF}
        self.operation_mode = operation_modes.get(self.operation_mode_str, IMU)

        oscillators = {'EXTERNAL': EXTERNAL_OSCILLATOR, 'INTERNAL': INTERNAL_OSCILLATOR}
        self.oscillator = oscillators.get(self.oscillator_str, INTERNAL_OSCILLATOR)
        self.imu = imu
        # self.imu = BoschIMU(port=self.serial_port)
        self.initial_yaw = 0.0
        self.initial_pitch = 0.0
        self.initial_roll = 0.0
        self.initialized = False

        self.relative_yaw = 0.0
        self.relative_pitch = 0.0
        self.relative_roll = 0.0

        self.last_update_time = time.time()

        self.imu_msg = imu_msg

        self.set_imu_configuration()

    ## Sets IMU configuration, including measurement units and operation mode
    def set_imu_configuration(self):
        self.imu.enable_imu_configuration()
        self.imu.set_imu_units(
            acceleration_units=METERS_PER_SECOND,
            angular_velocity_units=RAD_PER_SECOND,
            euler_orientation_units=RAD,
            temperature_units=CELSIUS,
            orientation_mode=WINDOWS_ORIENTATION
        )
        self.imu.set_imu_axis(axis_placement=P1)
        self.imu.set_oscillator(oscillator_type=self.oscillator)
        self.imu.set_imu_operation_mode(operation_mode=self.operation_mode)

    ## Determines if the IMU data needs to be updated based on a fixed update frequency
    #  @retval bool True if the IMU should update, False otherwise
    def should_update_imu(self):
        current_time = time.time()
        if current_time - self.last_update_time >= 0.025:
            self.last_update_time = current_time
            return True
        return False

    ## Updates the yaw, pitch, and roll angles of the IMU sensor
    def update_angles(self):
        quaternion = self.imu.get_quaternion_orientation()
        current_yaw, current_pitch, current_roll = self.calculate_yaw_pitch_roll(quaternion)

        if not self.initialized:
            self.initial_yaw = current_yaw
            self.initial_pitch = current_pitch
            self.initial_roll = current_roll
            self.initialized = True

        self.relative_yaw = current_yaw - self.initial_yaw
        self.relative_pitch = current_pitch - self.initial_pitch
        self.relative_roll = current_roll - self.initial_roll

    ## Fetches updated IMU data from the sensor
    def update_imu(self):
        self.imu.update_imu_data()
        self.update_angles()

    ## Converts an angle to a value within 0 to 360 degrees
    #  @param angle Input angle in degrees
    #  @retval float Converted angle within 0 to 360 degrees
    def convert_angle(self, angle):
        return (angle + 360) % 360

    ## Calculates yaw, pitch, and roll from quaternion data
    #  @param quaternion Quaternion tuple (x, y, z, w)
    #  @retval tuple (yaw, pitch, roll) in radians
    def calculate_yaw_pitch_roll(self, quaternion):
        z, y, x, w = quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm > 0:
            x /= norm
            y /= norm
            z /= norm
            w /= norm
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        return yaw, pitch, roll

    ## Generates the ROS IMU message with the updated sensor data
    #  @retval Imu Updated IMU message ready for publishing
    def get_imu_message(self):
        if self.should_update_imu():
            self.update_imu()

        quaternion = self.imu.get_quaternion_orientation()
        linear_acceleration = self.imu.get_linear_acceleration()
        gyroscope = self.imu.get_gyroscope()

        # Reuse the imu_msg attribute
        self.imu_msg.header.frame_id = self.frame_id
        # self.imu_msg.header.stamp = rospy.get_time()
        
        self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w = quaternion
        self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z = linear_acceleration
        self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z = gyroscope

        # Set covariance values to -1
        self.imu_msg.orientation_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1
        self.imu_msg.angular_velocity_covariance[0] = -1
        
        return self.imu_msg

    ## Fetches the current relative yaw value
    #  @retval tuple Relative yaw in radians and degrees
    def get_yaw(self):
        if self.should_update_imu():
            self.update_imu()
        return self.relative_yaw, self.convert_angle(math.degrees(self.relative_yaw))
    ## Fetches the current relative pitch value
    #  @retval tuple Relative pitch in radians and degrees
    def get_pitch(self):
        if self.should_update_imu():
            self.update_imu()
        return self.relative_pitch, self.convert_angle(math.degrees(self.relative_pitch))
    ## Fetches the current relative roll value
    #  @retval tuple Relative roll in radians and degrees
    def get_roll(self):
        if self.should_update_imu():
            self.update_imu()
        return self.relative_roll, self.convert_angle(math.degrees(self.relative_roll))

## @class ROSIMU
#  @brief Extends SensorIMU to publish data as a ROS node.
#
#  Manages publishing the IMU data as ROS messages on the topic 'imu/data'.
class ROSIMU(SensorIMU):
    ## Constructor for initializing ROSIMU instance
    #  @param imu Reference to the BoschIMU device object
    #  @param imu_msg Reference to the IMU device object
    def __init__(self,imu,imu_msg):
        super().__init__(imu,imu_msg)
        self.node_name = rospy.get_name()
        self.pub_imu_msg = rospy.Publisher('imu/data', Imu, queue_size=1)
        self.imu_msg_seq_counter = 0
    ## Publishes the IMU message to the 'imu/data' topic
    def publish_imu(self):
        self.get_imu_message()
        self.imu_msg.header.seq = self.imu_msg_seq_counter
        self.pub_imu_msg.publish(self.imu_msg)
        self.imu_msg_seq_counter += 1
