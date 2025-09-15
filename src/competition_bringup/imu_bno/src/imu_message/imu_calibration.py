#!/usr/bin/env python3

import time
import os
from imu_message.imu_bno055_api import * 

NOT_CALIBRATED = 0x00
FULL_CALIBRATION = 0x01

class CalibrationIMU:

    def __init__(self, serial_port="/dev/ttyUSB0", operation_mode_str="IMU"):
        self.serial_port = serial_port
        self.operation_mode_str = operation_mode_str

        # Map operation modes to constants
        switcher = {
            'IMU': IMU,
            'COMPASS': COMPASS,
            'M4G': M4G,
            'NDOF_FMC_OFF': NDOF_FMC_OFF,
            'NDOF': NDOF,
        }
        self.operation_mode = switcher.get(self.operation_mode_str, IMU)

        # Create an IMU instance
        self.bno055 = BoschIMU(port=self.serial_port)
        self.calibration_full_counter = 0

    def init_calibration(self):
        print("=============================================================")
        print(f"The IMU will be calibrated to work in {self.operation_mode_str} mode")
        print("=============================================================")

    def calibrate_imu(self):
        is_imu_calibrated = NOT_CALIBRATED
        calibration_status, status = self.bno055.calibrate_imu(self.operation_mode)

        if status == RESPONSE_OK:
            system_calibration_status = calibration_status[0]
            gyroscope_calibration_status = calibration_status[1]
            accelerometer_calibration_status = calibration_status[2]
            magnetometer_calibration_status = calibration_status[3]

            if self.operation_mode in [NDOF_FMC_OFF, NDOF]:
                print(f"[System: {system_calibration_status}] [Gyroscope: {gyroscope_calibration_status}] "
                      f"[Accelerometer: {accelerometer_calibration_status}] [Magnetometer: {magnetometer_calibration_status}]")
                if all(x == 3 for x in calibration_status):
                    self.calibration_full_counter += 1

            elif self.operation_mode == IMU:
                print(f"[Gyroscope: {gyroscope_calibration_status}] [Accelerometer: {accelerometer_calibration_status}]")
                if gyroscope_calibration_status == 3 and accelerometer_calibration_status == 3:
                    self.calibration_full_counter += 1

            elif self.operation_mode in [COMPASS, M4G]:
                print(f"[Accelerometer: {accelerometer_calibration_status}] [Magnetometer: {magnetometer_calibration_status}]")
                if accelerometer_calibration_status == 3 and magnetometer_calibration_status == 3:
                    self.calibration_full_counter += 1

            if self.calibration_full_counter >= 3:
                is_imu_calibrated = FULL_CALIBRATION
                print("IMU successfully calibrated!")

            time.sleep(1)

        return is_imu_calibrated

    def read_calibration(self):
        calibration, status = self.bno055.get_calibration()
        if status != RESPONSE_OK:
            print("Error: Unable to read IMU calibration")
        return calibration

    def write_calibration(self, calibration):
        status = self.bno055.set_calibration(calibration)
        if status == RESPONSE_OK:
            print("Calibration successfully written to the IMU")
            self.save_calibration_in_file(calibration)
        else:
            print("Error: Unable to write calibration to IMU")

    def save_calibration_in_file(self, calibration):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        try:
            with open(os.path.join(dir_path, f"{self.operation_mode_str}_calibration"), "wb") as binary_file:
                binary_file.write(calibration)
            print("Calibration successfully saved in binary file")
        except Exception as e:
            print(f"Error while saving calibration to file: {e}")

    def read_calibration_from_file(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        try:
            with open(os.path.join(dir_path, "calibration"), "rb") as binary_file:
                calibration_data = binary_file.read()
            print("Calibration data read from file:")
            print(calibration_data)
        except Exception as e:
            print(f"Error reading calibration file: {e}")
            calibration_data = None
        return calibration_data

    def run(self):
        self.init_calibration()
        while True:
            status = self.calibrate_imu()
            if status == FULL_CALIBRATION:
                calibration_data = self.read_calibration()
                self.write_calibration(calibration_data)
                print("Calibration completed successfully. Exiting...")
                break


if __name__ == '__main__':
    imu_calibration = CalibrationIMU()
    try:
        imu_calibration.run()
    except KeyboardInterrupt:
        print("Calibration interrupted by user.")
