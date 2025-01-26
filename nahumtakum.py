from machine import I2C, Pin, PWM,SPI
from mpu6050 import MPU6050  # Assuming the MPU6050 module is named mpu6050.py
from time import sleep_ms, ticks_ms, ticks_diff, sleep
import motordriver
import utime
from ili9341 import Display, color565
from xglcd_font import XglcdFont
import sdcard
import os
import math
from bmphandle import read_bmp_file, display_bmp, read_bmp_in_chunks

# visualization settings
espresso_dolce = XglcdFont('fonts/EspressoDolce18x24.c', 18, 24)
arcade = XglcdFont('fonts/ArcadePix9x11.c', 9, 11)

# Initialize SD card (optional, can be removed if not needed)
class NahumTakum:
    def __init__(self, in1, in2, ena, i2c_instance=None, encoder1_pin=None, encoder2_pin=None, wheel_size=None, display=None, font=None):
        # Motor control pins
        self._in1 = in1
        self._in2 = in2
        self._ena = ena

        # Encoder pins
        self.encoder1_pin = encoder1_pin
        self.encoder2_pin = encoder2_pin

        # Additional attributes
        self.wheel_size = wheel_size
        self.i2c = i2c_instance  # Store the I²C instance for use in the class

        # SPI and display setup
        self.spi = SPI(1, baudrate=33000000, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
        self.dc = Pin(25)
        self.cs = Pin(5)  # Screen CS
        self.rst = Pin(33)
        self.display = Display(self.spi, cs=self.cs, dc=self.dc, rst=self.rst, rotation=0, mirror=False)
        self.display.clear()
        self.sd_cs = Pin(15)

        # Load fonts
        self.arcade_font = XglcdFont('fonts/ArcadePix9x11.c', 9, 11)


        # Initialize motor driver object
        self.motor = motordriver.motordriver(
            encoder1_pin=self.encoder1_pin,
            encoder2_pin=self.encoder2_pin,
            in1_pin=self._in1,
            in2_pin=self._in2,
            wheel_size=self.wheel_size,
            display=self.display,
            font=self.arcade_font,  # Use the internally loaded font
        )
                
        # Initialize MPU6050
        if i2c_instance is None:
            raise ValueError("I2C instance is required for MPU6050 initialization.")

        self.mpu = MPU6050(i2c_instance)
        
        self.last_time = ticks_ms()  # Record the last time
        self.current_angle = 0.0  # Start with an initial angle
        # PID controller variables
        self.current_time = ticks_ms()
        self.previous_time = self.current_time
        self.error = 0
        self.last_error = 0
        self.cum_error = 0
        self._kp = 1.0
        self._ki = 0.0
        self._kd = 0.0
        self.debug = False

    def begin(self):
        self.display.draw_text(40, 20, "NahumTakum!", espresso_dolce, color565(100, 255, 255))
        self.display.draw_text(50, 100, "initializing...", espresso_dolce, color565(80, 255, 255))
        self.stop()
        # Initialize MPU6050
        self.mpu.calibrate_gyro
        self.previous_time = ticks_ms()
        print("MPU6050 initialized.")
        self.display.clear()

    def run(self):
        # Update MPU6050 readings
        self.mpu.update_angle()
        
    def ang(self):
        return(self.mpu.update_angle())
    def stop(self):
        self.motor.motgo(0)
    def motgo(self, speed):
        self.motor.motgo(speed)    
    def gotoang(self, deg):
        self.motor.godegreesp(deg, 480, 0.5, 0.4, 0, color565(255, 255, 0), 0)
    def clearscreen(self):
        self.display.clear()
    def display_pid_values(self, kp, ki, kd, color, Line_index):
        self.motor.display_pid_values(kp, ki, kd, color, Line_index)
    
    def tumble(self, kp, ki, kd, color):
        """
        Performs motor control using PID, limited by safety boundaries.
        """
        # Set PID constants
        self._kp = kp
        self._ki = ki
        self._kd = kd

        # Read current motor angle
        self.ang = self.motor.degrees  # this is updated via an IRQ

        if -130 < self.ang < 130:  # Safety measure
            #print('In boundaries')

            # Current angle as Process Value (PV)
            pitch = self.ang  

            # Calculate PID error and output
            inp = self.motor.PIDcalc(
                0,  # Target angle
                pitch,
                self._kp,
                self._ki,
                self._kd,
                color,  #Color for plotting PID response
                True, #plot flag
            )

            # Limit the speed if necessary
            speed = int(inp)
            #print(f"Calculated speed: {speed}")

            # Command motor to move at calculated speed
            self.motor.motgo(speed)

            # Optional: Log motor angle
            #print(f"Current angle: {self.motor.degrees}")

            return speed
        else:
            # Out of safety boundaries
            print('Out of boundaries! Stopping motor.')

            # Optional: Stop motor or take necessary safety measures
            self.motor.motgo(0)
            return 0
    
    def testtumble(self, kp, ki, kd, Line_index):
        #Sort out the colors
        if Line_index == 0:
            color = color565(255, 255, 0)  # Yellow
        elif Line_index == 1:
            color = color565(30, 255, 30)  # Green
        elif Line_index == 2:
            color = color565(0, 255, 255)  # Cyan
        elif Line_index == 3:
            color = color565(255, 0, 255)  # Magenta
        else:  # Default case
            color = color565(255, 255, 255)  # White (fallback)
        # Set PID constants
        
        
        for _ in range(480):
            self.run()
            self.tumble(kp,ki,kd, color)
        self.stop()
        self.display_pid_values(kp, ki, kd, color, Line_index)
       

