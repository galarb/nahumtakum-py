from machine import I2C, Pin
from nahumtakum import NahumTakum
import time
from math import degrees, atan2

i2c = I2C(1, scl=Pin(16), sda=Pin(17))  # running out of pins...

# Create NahumTakum instance

robot = NahumTakum(
    in1=4,
    in2=0,
    ena=13,
    encoder1_pin=14,
    encoder2_pin=12,
    wheel_size=65,
    i2c_instance=i2c  # Pass the I²C instance

)
# Initialize the MPU6050 and system
robot.begin()

#robot.motgo(-90)
#time.sleep(2)
#robot.stop()
#robot.testtumble(kp,ki,kd,lineindex)
robot.gotoang(-90)

robot.testtumble(1, 2, 0, 1)
time.sleep(2)

'''robot.gotoang(-90)
robot.testtumble(2, 0, 0.1, 2)
time.sleep(2)'''
robot.stop()


# Main loop
#angle_y = robot.mpu.update_angle()
    

