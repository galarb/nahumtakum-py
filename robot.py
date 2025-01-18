from machine import I2C, Pin
from nahumtakum import NahumTakum
import time
from math import degrees, atan2

# I2C setup (Adjust pins for your ESP32 setup)
i2c = I2C(1, scl=Pin(22), sda=Pin(21))  # Example I2C pins

# Create NahumTakum instance
robot = NahumTakum(in1=25, in2=26, ena=27, i2c_instance=i2c)

# Initialize the MPU6050 and system
robot.begin()

# Calibration settings (Adjust these based on your calibration process)
gyro_z_offset = -1.5  # Example offset; calibrate this
gyro_sensitivity = 131  # Example: degrees/s per unit


# Main loop
while True:
    #angle_y = robot.mpu.update_angle()
    robot.run()
    robot.tumble(1,1,0)

