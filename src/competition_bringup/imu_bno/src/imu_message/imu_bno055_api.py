#!/usr/bin/env python3


import serial #pyserial
import math
import time
import sys, os

# Enable print to debug
ENABLE_DEBUG = False

# IMU identification
CHIP_ID = 0x00                          # Returns IMU ID
BNO055_ID = 0xA0                        # All BNO055 has same ID

# Command utils (see 4.7)
START_BYTE = 0xAA
WRITE = 0x00
READ = 0x01
RESPONSE_HEADER = 0xEE
WRITE_SUCCESS = 0x01
READ_SUCCESS = 0xBB
READ_FAIL = 0x02
WRITE_FAIL = 0x03
REGMAP_INVALID_ADDRESS = 0x04 
REGMAP_WRITE_DISABLED = 0x05
WRONG_START_BYTE = 0x06
BUS_OVER_RUN_ERROR = 0x7
MAX_LENGTH_ERROR = 0x08
MIN_LENGTH_ERROR = 0x09
RECEIVE_CHARACTER_TIMEOUT = 0x0A

# Custom command utils
RESPONSE_OK  = 0x0B
RESPONSE_ERROR = 0x0C

# IMU configurations

# System configuration
SYS_TRIGGER = 0x3F
RST_SYS = 0x20
CLK_SEL = 0x80
SYS_CLK_STATUS = 0x38
INTERNAL_OSCILLATOR = 0x00
EXTERNAL_OSCILLATOR = 0x01


# Power configuration (section 3.2)
PWR_MODE = 0x3E                         # Set power mode
NORMAL_MODE = 0x00                      # Normal power
LOW_POWER_MODE = 0x1                    # Low power

# Unit configuration (section 3.6.1)
UNIT_SEL = 0x3B                         # Set units
METERS_PER_SECOND   = 0b00000000        # Acceleration
MILI_G              = 0b00000001        # Acceleration units
DEG_PER_SECOND      = 0b00000000        # Anguar velocity units
RAD_PER_SECOND      = 0b00000010        # Anguar velocity units
DEG                 = 0b00000000        # Euler orientation units
RAD                 = 0b00000100        # Euler orientation units
CELSIUS             = 0b00000000        # Temperature units 
FAHRENHEIT          = 0b00001000        # Temperature units
WINDOWS_ORIENTATION = 0b00000000        # Orientation mode      
ANDROID_ORIENTATION = 0b10000000        # Orientation mode

# Operational mode configuration (section 3.3 table 3.5)
OPR_MODE =  0x3D                        # Set operation mode
CONFIGMODE = 0x00                       # Enable configuration
IMU = 0x08                              # Relative orientation, magnetometer will suspend
COMPASS = 0x09                          # Absolute orientation, gyroscope will suspend
M4G = 0x0A                              # Relative orientation, gyroscope will suspend
NDOF_FMC_OFF = 0x0B                     # Absolute orientation, all sensors will work
NDOF = 0x0C                             # Absolute orientation, all sensors will work

# Axis configuration
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

AXIS_REMAP_CONFIG_P0 = 0x21
AXIS_REMAP_CONFIG_P1 = 0x24
AXIS_REMAP_CONFIG_P2 = 0x24
AXIS_REMAP_CONFIG_P3 = 0x21
AXIS_REMAP_CONFIG_P4 = 0x24
AXIS_REMAP_CONFIG_P5 = 0x21
AXIS_REMAP_CONFIG_P6 = 0x21
AXIS_REMAP_CONFIG_P7 = 0x24

AXIS_REMAP_SIGN_P0 = 0x04
AXIS_REMAP_SIGN_P1 = 0x00
AXIS_REMAP_SIGN_P2 = 0x06
AXIS_REMAP_SIGN_P3 = 0x02
AXIS_REMAP_SIGN_P4 = 0x03
AXIS_REMAP_SIGN_P5 = 0x01
AXIS_REMAP_SIGN_P6 = 0x07
AXIS_REMAP_SIGN_P7 = 0x05

P0 = 0x00
P1 = 0x01
P2 = 0x02
P3 = 0x03
P4 = 0x04
P5 = 0x05
P6 = 0x06
P7 = 0x07



# Each vector is accessible by indicating the first register of its axis
VECTOR_ALL_DATA = 0x08                   # First axis register is ACC_DATA_X_LSB
VECTOR_ACCELERATION = 0x08               # First axis register is ACC_DATA_X_LSB 
VECTOR_GYROSCOPE = 0x14                  # First axis register is GYR_DATA_X_LSB 
VECTOR_QUATERNION_ORIENTATION = 0x20     # First axis register is QUA_DATA_W_LSB 
VECTOR_LINEAR_ACCELERATION = 0x28        # First axis register is LIA_DATA_X_LSB 
VECTOR_EULER_ORIENTATION = 0x1A          # First axis register is EUL_DATA_X_LSB 
VECTOR_MAGNETOMETER = 0x0E               # First axis register is MAG_DATA_X_LSB 
VECTOR_GRAVITY = 0x2E                    # First axis register is GRV_DATA_X_LSB
TEMPERATURE = 0X34                       # Register is TEMP 

# Each axis component has 2 bytes
VECTOR_ALL_DATA_LENGTH = 46              # acc, gyr, qua, linacc, eu, mag, grav and temp
VECTOR_ACCELERATION_LENGTH = 6           # x,y,z  (2 bytes x 3 axis)
VECTOR_GYROSCOPE_LENGTH = 6              # roll, yaw, pitch (2 bytes x 3 axis)
VECTOR_QUATERNION_ORIENTATION_LENGTH = 8 # w,x,y,z (2 bytes x 4 axis)
VECTOR_LINEAR_ACCELERATION_LENGTH = 6    # x,y,z  (2 bytes x 3 axis)
VECTOR_EULER_ORIENTATION_LENGTH = 6      # roll, yaw, pitch (2 bytes x 3 axis)
VECTOR_MAGNETOMETER_LENGTH = 6           # x,y,z  (2 bytes x 3 axis)
VECTOR_GRAVITY_LENGTH = 6                # x,y,z  (2 bytes x 3 axis)
TEMPERATURE_LENGTH = 1                   # temperature (1 byte)


# Convert the value received from the imu into real units (see section 3.6.4)
# The rest of values do not need conversion
ANGULAR_RAD_SCALE = 900.0                # 1 rad/s = 900 LSB
ANGULAR_DEG_SCALE = 16.0                 # 1 rad/s = 16 LSB
LINEAR_SCALE = 100.0                     # 1 m/s2 = 100 LSB 
MAGNETOMETER_SCALE = 16.0                # 1 uT = 16 LSB
GRAVITY_SCALE = 100.0                    # 1 m/s2 = 100 LSB
TEMPERATURE_F_SCALE = 2.0                # 2 F = 1 LSB



class BoschIMU:

    serial_port = serial.Serial()

    def __init__(self, port = "/dev/ttyUSB0"):
        
        self.is_configuration_enabled = False
        self.operation_mode = -1

        # These units are modified internally set_imu_units()
        self.acceleration_units = int()
        self.angular_velocity_units = int()
        self.euler_orientation_units = int()
        self.temperature_units = int()
        self.orientation_mode = int()

        self.raw_accelerometer = []
        self.raw_magnetometer = []
        self.raw_gyroscope = []
        self.raw_euler = []
        self.raw_quaternion = []
        self.raw_linear_acceleration = []
        self.raw_gravity = []
        self.raw_temperature = 0

        # The BNO055 supports UART interface with the following settings:
        # 115200 bps, 8N1 (8 data bits, no parity bit, one stop bit)

        self.serial_port = serial.Serial(port = port,
                                         baudrate = 115200,
                                         timeout = 0.1,
                                         bytesize = serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits= serial.STOPBITS_ONE
                                        )


    # Returns number of bytes used by a INT variable
    def int_byte_size(self, value):

        if value != 0:
            valueBits = int(math.log(value, 2) + 1)

            if valueBits % 8 != 0:
                number_of_bytes = (valueBits / 8) + 1
            else:
                number_of_bytes = valueBits / 8
        else:
            number_of_bytes = 1
            

        return int(number_of_bytes)


    def _print(self, msg):

        if ENABLE_DEBUG == True:
            print(msg)

    def build_write_command(self, address, data):

        try:
            data_length = self.int_byte_size(data) # Returns the number of bytes used by int variable
        except:
            data_length = len(data)
 
        command = bytearray()
        command.append(START_BYTE)
        command.append(WRITE)
        command.append(address)
        command.append(data_length)            # Size of the data to write 
        
        try:
            command.append(data)
        except:
            command.extend(data)            

        return command


    def build_read_command(self, address, response_data_length):

        command = bytearray()
        command.append(START_BYTE)
        command.append(READ)
        command.append(address)
        command.append(response_data_length)   # Size of the expected response data

        return command


    def check_response(self, response):

        state = RESPONSE_ERROR

        if len(response) > 1:

            response_header = response[0]
            response_status = response[1]

            if response_header == RESPONSE_HEADER:
                
                if response_status == WRITE_SUCCESS:
                    self._print("Write succes")
                    state = RESPONSE_OK

                if response_status == READ_FAIL:
                    self._print("Read fail")
                    state = RESPONSE_ERROR

                if response_status == WRITE_FAIL:  
                    self._print("Write fail")
                    state = RESPONSE_ERROR

                if response_status == REGMAP_INVALID_ADDRESS:
                    self._print("Regmap invalid address")
                    state = RESPONSE_ERROR

                if response_status == REGMAP_WRITE_DISABLED:
                    self._print("Regmap write disabled")
                    state = RESPONSE_ERROR

                if response_status == WRONG_START_BYTE:
                    self._print("Wrong start byte")
                    state = RESPONSE_ERROR

                if response_status == BUS_OVER_RUN_ERROR:
                    self._print("Bus over run error")
                    state = RESPONSE_ERROR

                if response_status == MAX_LENGTH_ERROR:
                    self._print("Max length error")
                    state = RESPONSE_ERROR

                if response_status ==  MIN_LENGTH_ERROR:
                    self._print("Min length error")
                    state = RESPONSE_ERROR

                if response_status == RECEIVE_CHARACTER_TIMEOUT:
                    self._print("Receive character timeout")
                    state = RESPONSE_ERROR

            elif response_header == READ_SUCCESS:
                self._print("Read success")
                state = RESPONSE_OK

            else:
                self._print("Response header not detected")
                state = RESPONSE_ERROR

        else:
            self._print("Timeout expired: data not received")
            state = RESPONSE_ERROR

        return state
       

    def write_imu(self, address, data):
        
        status = -1
        attempts = 0

        while status != RESPONSE_OK:

            # Write register with data value
            command = self.build_write_command(address, data) 
            self.serial_port.write(command)

            # Get serial response 
            command_length = 2   # Response is always made up of two bytes
            response = self.serial_port.read(command_length)
            
            # Check response
            status = self.check_response(response)

            attempts+=1
            if attempts >= 10:
                self._print("Error, after ten attempts the response was not received correctly")
                status = RESPONSE_ERROR
                break

        

        return response, status


    def read_imu(self, address, response_data_length):

        # Read register
        command = self.build_read_command(address, response_data_length) 
        self.serial_port.write(command)

        # Get serial header response
        header_response = self.serial_port.read(2)
        status = self.check_response(header_response)

        # If status it is OK, get the rest of response
        if status == RESPONSE_OK:       
            response_data = self.serial_port.read(response_data_length) 
            response = header_response + response_data
        else:
            response = 0
        return response, status


    def enable_imu_configuration(self):
        status = self.set_imu_operation_mode(CONFIGMODE)
        return status
    def set_imu_units(self, acceleration_units, angular_velocity_units,
                      euler_orientation_units, temperature_units, orientation_mode ):
        status = -1
        if self.is_configuration_enabled == True:
            self.acceleration_units = acceleration_units  
            self.angular_velocity_units = angular_velocity_units 
            self.euler_orientation_units = euler_orientation_units          
            self.temperature_units = temperature_units             
            self.orientation_mode = orientation_mode  

            response, status = self.write_imu(UNIT_SEL, self.acceleration_units |    
                                                        self.angular_velocity_units |     
                                                        self.euler_orientation_units  |                
                                                        self.temperature_units |             
                                                        self.orientation_mode   
                                                        )
        else:
            self._print("Operation mode 'CONFIGMODE' is not set!")  
            status = RESPONSE_ERROR
        return status
    def set_imu_axis(self, axis_placement):
        status = -1
        if self.is_configuration_enabled == True:

            config_switcher = {
                    P0: AXIS_REMAP_CONFIG_P0,
                    P1: AXIS_REMAP_CONFIG_P1,
                    P2: AXIS_REMAP_CONFIG_P2,
                    P3: AXIS_REMAP_CONFIG_P3,
                    P4: AXIS_REMAP_CONFIG_P4,
                    P5: AXIS_REMAP_CONFIG_P5,
                    P6: AXIS_REMAP_CONFIG_P6,
                    P7: AXIS_REMAP_CONFIG_P7
                }
            sign_switcher = {
                    P0: AXIS_REMAP_SIGN_P0,
                    P1: AXIS_REMAP_SIGN_P1,
                    P2: AXIS_REMAP_SIGN_P2,
                    P3: AXIS_REMAP_SIGN_P3,
                    P4: AXIS_REMAP_SIGN_P4,
                    P5: AXIS_REMAP_SIGN_P5,
                    P6: AXIS_REMAP_SIGN_P6,
                    P7: AXIS_REMAP_SIGN_P7
            }
            remap_config = config_switcher.get(axis_placement, P2)
            remap_sign = sign_switcher.get(axis_placement, P2)
            response_remap_config, status_config = self.write_imu(AXIS_MAP_CONFIG, remap_config)
            response_remap_sign, status_sign = self.write_imu(AXIS_MAP_SIGN, remap_sign)
            if status_config == RESPONSE_OK and status_sign == RESPONSE_OK:
                status = RESPONSE_OK
            else:
                status = RESPONSE_ERROR
        else:
            self._print("Operation mode 'CONFIGMODE' is not set!")
            status = RESPONSE_ERROR
        return status
    def set_imu_operation_mode(self, operation_mode):   
        if self.operation_mode != operation_mode:
            self.operation_mode = operation_mode
            response, status = self.write_imu(OPR_MODE, operation_mode)
            if status == RESPONSE_OK:
               
                if operation_mode == CONFIGMODE:
                    self.is_configuration_enabled = True
                else:
                    self.is_configuration_enabled = False
            time.sleep(0.2) # Sleep 200 ms
        else:
            status = RESPONSE_OK
        return status
    def set_oscillator(self, oscillator_type):
        status = -1
        response_clk, status_clk = self.read_imu(SYS_CLK_STATUS, 1)
        if status_clk == RESPONSE_OK:
            main_clock_status = response_clk[2] 
            if main_clock_status == 0:                
                    response, status = self.write_imu(SYS_TRIGGER, oscillator_type) 
            else:
                status = RESPONSE_ERROR
                self._print("Main clock not available")
        else:
            status = RESPONSE_ERROR
        return  status




    def update_imu_data(self):
        response, status = self.read_imu(VECTOR_ALL_DATA, VECTOR_ALL_DATA_LENGTH)
        if status == RESPONSE_OK:
            self.raw_accelerometer = response[2:8]
            self.raw_magnetometer = response[8:14]
            self.raw_gyroscope = response[14:20]
            self.raw_euler = response[20:26]
            self.raw_quaternion = response[26:34]
            self.raw_linear_acceleration = response[34:40]
            self.raw_gravity = response[40:46]
            self.raw_temperature = response[46:47]
    def get_quaternion_orientation(self):
        response = self.raw_quaternion
        raw_quaternion_w = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_quaternion_x = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_quaternion_y = int.from_bytes(response[4:6], 'little',  signed= True) 
        raw_quaternion_z = int.from_bytes(response[6:8], 'little',  signed= True) 
        quaternion_w = raw_quaternion_w
        quaternion_x = raw_quaternion_x
        quaternion_y = raw_quaternion_y
        quaternion_z = raw_quaternion_z
        return quaternion_w, quaternion_x, quaternion_y, quaternion_z
    def get_euler_orientation(self):
        response = self.raw_euler
        raw_euler_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_euler_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_euler_z = int.from_bytes(response[4:6], 'little',  signed= True)
        if self.euler_orientation_units == RAD:
            euler_x = raw_euler_x / ANGULAR_RAD_SCALE
            euler_y = raw_euler_y / ANGULAR_RAD_SCALE
            euler_z = raw_euler_z / ANGULAR_RAD_SCALE
        elif self.euler_orientation_units == DEG:
            euler_x = raw_euler_x / ANGULAR_DEG_SCALE
            euler_y = raw_euler_y / ANGULAR_DEG_SCALE
            euler_z = raw_euler_z / ANGULAR_DEG_SCALE
        else:
            self._print("Error: wrong angle unit, you can use: RAD or DEG ")
            return 
        return euler_x, euler_y, euler_z
    def get_gyroscope(self):
        response = self.raw_gyroscope
        raw_gyroscope_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_gyroscope_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_gyroscope_z = int.from_bytes(response[4:6], 'little',  signed= True)
        if self.angular_velocity_units == RAD_PER_SECOND:
            gyroscope_x = raw_gyroscope_x / ANGULAR_RAD_SCALE
            gyroscope_y = raw_gyroscope_y / ANGULAR_RAD_SCALE
            gyroscope_z = raw_gyroscope_z / ANGULAR_RAD_SCALE
        elif self.angular_velocity_units == DEG_PER_SECOND:
            gyroscope_x = raw_gyroscope_x / ANGULAR_DEG_SCALE
            gyroscope_y = raw_gyroscope_y / ANGULAR_DEG_SCALE
            gyroscope_z = raw_gyroscope_z / ANGULAR_DEG_SCALE
        else:
            self._print("Error: wrong angle unit, you can use: RAD or DEG ")
            return 
        return gyroscope_x, gyroscope_y, gyroscope_z
    def get_linear_acceleration(self):
        response = self.raw_linear_acceleration
        raw_linear_acceleration_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_linear_acceleration_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_linear_acceleration_z = int.from_bytes(response[4:6], 'little',  signed= True)
        if self.acceleration_units == METERS_PER_SECOND:
            linear_acceleration_x = raw_linear_acceleration_x / LINEAR_SCALE
            linear_acceleration_y = raw_linear_acceleration_y / LINEAR_SCALE
            linear_acceleration_z = raw_linear_acceleration_z / LINEAR_SCALE
        elif self.acceleration_units == MILI_G:
            linear_acceleration_x = raw_linear_acceleration_x
            linear_acceleration_y = raw_linear_acceleration_y
            linear_acceleration_z = raw_linear_acceleration_z
        else:
            self._print("Error: wrong angle unit, you can use: METERS_PER_SECOND or MILI_G ")
            return 
        return linear_acceleration_x, linear_acceleration_y, linear_acceleration_z