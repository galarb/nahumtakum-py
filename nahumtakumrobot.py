from machine import I2C, Pin
from nahumtakum import NahumTakum
import time
from math import degrees, atan2

i2c = I2C(1, scl=Pin(16), sda=Pin(17))  # running out of pins...

# Create NahumTakum instance

robot = NahumTakum(
    in1=0,
    in2=4,
    ena=13,
    encoder1_pin=14,
    encoder2_pin=12,
    wheel_size=65,
    i2c_instance=i2c  # Pass the I²C instance

)
# Initialize the MPU6050 and system
robot.begin()

robot.gotoang(90)
for _ in range(1000):
    robot.run()
    robot.tumble(1,0.01,0)
    robot.stop()
# Main loop
#angle_y = robot.mpu.update_angle()
    

