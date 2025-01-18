from machine import I2C, Pin
from nahumtakum import NahumTakum
import time

# I2C setup (Adjust pins for your ESP32 setup)
i2c = I2C(1, scl=Pin(22), sda=Pin(21))  # Example I2C pins

# Create NahumTakum instance
robot = NahumTakum(in1=25, in2=26, ena=27, i2c_instance=i2c)

# Initialize the MPU6050 and system
robot.begin()
gyro_data = robot.mpu.update_angle()
while True:
    angle_y = robot.mpu.update_angle()
    print(angle_y)
    time.sleep_ms(100)  # Delay for stability
