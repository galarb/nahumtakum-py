from machine import I2C, Pin, PWM
from mpu6050 import MPU6050  # Assuming the MPU6050 module is named mpu6050.py
from time import sleep_ms, ticks_ms, ticks_diff


class NahumTakum:
    def __init__(self, in1, in2, ena, i2c_instance=None):
        # Motor control pins
        self._in1 = in1
        self._in2 = in2
        self._ena = ena

        # Setup motor pins
        self.ena_pwm = Pin(self._ena, Pin.OUT)
        self.PWM_in1 = PWM(Pin(self._in1))
        self.PWM_in1.freq(1000)  # Set frequency to 1kHz
        self.PWM_in2 = PWM(Pin(self._in2))
        self.PWM_in2.freq(1000)  # Set frequency to 1kHz
        
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
        # Initialize MPU6050
        
        self.mpu.calibrate_gyro
        self.previous_time = ticks_ms()
        print("MPU6050 initialized.")


    def run(self):
        # Update MPU6050 readings
        self.mpu.update_angle()
        
    def ang(self):
        return(self.mpu.update_angle())
    

    def pid_calc(self, sp, pv):
        # PID calculation logic
        self.current_time = ticks_ms()
        elapsed_time = ticks_diff(self.current_time, self.previous_time) / 1000.0

        self.error = sp - pv
        self.cum_error += self.error * elapsed_time
        rate_error = (self.error - self.last_error) / elapsed_time

        output = (
            self._kp * self.error
            + self._ki * self.cum_error
            + self._kd * rate_error
        )

        self.last_error = self.error
        self.previous_time = self.current_time

        # Clamp the output
        output = max(min(output, 100), -100)
        return output

    def tumble(self, kp, ki, kd):
        # Set PID constants
        self._kp = kp
        self._ki = ki
        self._kd = kd

        pitch = self.ang()
        inp = self.pid_calc(0, pitch)
        speed = int(inp) #this is the speed value
        print('speed =', speed)
        self.motgo(speed)

    def motgo(self, speed):
        pwm_value = int(min(max(abs(speed), 0), 100) * 10.23)  # Map 0 to 100 to 0 to 1023
        if speed > 0:
            # Forward direction
            self.PWM_in1.duty(pwm_value)
            self.PWM_in2.duty(0)
        elif speed < 0:
            # Reverse direction
            self.PWM_in1.duty(0)
            self.PWM_in2.duty(pwm_value)
        else:
            # Stop the motor
            self.PWM_in1.duty(0)
            self.PWM_in2.duty(0)    

