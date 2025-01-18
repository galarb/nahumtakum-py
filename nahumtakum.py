from machine import I2C, Pin
from mpu6050 import MPU6050  # Assuming the MPU6050 module is named mpu6050.py
from time import sleep_ms, ticks_ms, ticks_diff


class NahumTakum:
    def __init__(self, in1, in2, ena, i2c_instance=None):
        # Motor control pins
        self._in1 = in1
        self._in2 = in2
        self._ena = ena

        # Setup motor pins
        Pin(self._in1, Pin.OUT)
        Pin(self._in2, Pin.OUT)
        self.ena_pwm = Pin(self._ena, Pin.OUT)

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
        self.mpu.update()
        if self.debug:
            print(f"Pitch: {self.get_pitch()}")

    def get_pitch(self):
        # Get the pitch (Y-axis angle)
        #print(self.mpu.accel.xyz)
        #print(self.mpu.gyro.xyz)
        #print(self.mpu.mag.xyz)
        
        #print(self.mpu.accel.z)
        
        # Read angular velocity (degrees per second or radians per second)
        gyro_z = self.mpu.gyro.z
        accel_angle = self.mpu.accel.y  # Compute pitch from accelerometer

        alpha = 0.98  # Tuning parameter
        # Calculate time elapsed in seconds
        current_time = ticks_ms()
        elapsed_time = ticks_diff(current_time, self.last_time) / 1000.0  # Convert ms to seconds
        self.last_time = current_time

        # Integrate to find the angle
        self.current_angle += gyro_z * elapsed_time
        self.current_angle = alpha * (self.current_angle + gyro_z * elapsed_time) + (1 - alpha) * accel_angle

        # Optional: Limit the angle to a specific range (e.g., -180 to 180 degrees)
        self.current_angle = (self.current_angle + 180) % 360 - 180  # Wrap angle to [-180, 180]

        return self.current_angle

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
        output = max(min(output, 254), -254)
        return output

    def tumble(self, kp, ki, kd):
        # Set PID constants
        self._kp = kp
        self._ki = ki
        self._kd = kd

        pitch = self.get_pitch()
        inp = self.pid_calc(0, pitch)

        if inp < 0:
            Pin(self._in1).on()
            Pin(self._in2).off()
        else:
            Pin(self._in1).off()
            Pin(self._in2).on()

        pwm_value = int(abs(inp))
        self.ena_pwm.duty_u16(pwm_value)

