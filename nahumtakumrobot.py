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

robot.gotoangp(-90, 500, 0.85, 0.1, 0)
time.sleep(0.5)

robot.testtumble(1, 0, 0, 0)
time.sleep(0.5)

robot.gotoangp(-90, 500, 0.85, 0.1, 0)
time.sleep(0.5)

robot.testtumble(1.5, 1, 0, 1)
time.sleep(0.5)

robot.gotoangp(-90, 500, 0.85, 0.1, 0)
time.sleep(0.5)

robot.testtumble(1.1, 1.05, 0.1, 2)
time.sleep(0.5)

#print("Recorded processed:", robot.recorded_v())

'''try:
    while True:
        robot.run()  # Replace this with your actual method
except KeyboardInterrupt:
    print("Execution stopped by user.")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Perform any cleanup actions here
    print("Exiting the loop. Cleaning up...")
    # Example cleanup: stop motors, save data, etc.
    time.sleep(1)
    print('final ang:', robot.motang())
    robot.stop  # Stop the motor if applicable'''
# Main loop
#angle_y = robot.mpu.update_angle()
    

