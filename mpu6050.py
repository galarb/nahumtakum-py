from time import ticks_ms, ticks_diff, sleep_ms
class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        self.address = 0x68  # Default I2C address for MPU6050
        self.angle_z = 0  # Cumulative angle for Z-axis (yaw)
        self.bias_z = 0  # Gyroscope bias for Z-axis
        self.last_time = ticks_ms()  # Initialize time

        self.initialize_sensor()
        self.calibrate_gyro()  # Perform calibration at startup

    def initialize_sensor(self):
        # Wake up the MPU6050 (clear sleep mode)
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')  # Power management 1 register
        print("MPU6050 initialized.")

    def read_gyro_data(self):
        # Read gyroscope data (raw values)
        raw_data = self.i2c.readfrom_mem(self.address, 0x43, 6)  # Read 6 bytes starting at GYRO_XOUT_H
        return {
            "x": self.signedIntFromBytes(raw_data[0], raw_data[1]) / 131.0,  # Convert to deg/s
            "y": self.signedIntFromBytes(raw_data[2], raw_data[3]) / 131.0,
            "z": self.signedIntFromBytes(raw_data[4], raw_data[5]) / 131.0,
        }

    def signedIntFromBytes(self, byte1, byte2):
        # Combine two bytes and interpret as a signed integer
        value = (byte1 << 8) | byte2  # Combine the two bytes
        if value & (1 << 15):  # Check if the highest bit (sign bit) is set
            value -= (1 << 16)  # Convert to a negative value
        return value

    def calibrate_gyro(self):
        print("Calibrating gyroscope...")
        num_samples = 100
        #total_bias_x = 0
        total_bias_y = 0
        total_bias_z = 0

        for _ in range(num_samples):
            gyro_data = self.read_gyro_data()
            #total_bias_x += gyro_data["x"]
            total_bias_y += gyro_data["y"]
            #total_bias_z += gyro_data["z"]
            sleep_ms(10)  # Small delay between samples
            

        #self.bias_x = total_bias_x / num_samples
        self.bias_y = total_bias_y / num_samples
        self.bias_z = total_bias_z / num_samples
        #print(f"Calibration complete. Bias - X: {self.bias_x}, Y: {self.bias_y}, Z: {self.bias_z}")
        print(f"Calibration complete. Bias - Y: {self.bias_y}")

    def update_angle(self):
        # Get current time
        current_time = ticks_ms()
        dt = ticks_diff(current_time, self.last_time) / 1000.0  # Time in seconds
        self.last_time = current_time

        # Read gyroscope data (z-axis angular velocity)
        gyro_data = self.read_gyro_data()
        #angular_velocity_x = gyro_data["x"] - self.bias_x  # Subtract bias for X-axis
        angular_velocity_y = gyro_data["y"] - self.bias_y  # Subtract bias for Y-axis
        angular_velocity_z = gyro_data["z"] - self.bias_z  # Subtract bias for Z-axis
        
        #print(f"Angular velocities - X: {angular_velocity_x}, Y: {angular_velocity_y}, Z: {angular_velocity_z}")

        # Integrate angular velocity to find angle
        self.angle_z += angular_velocity_y * dt

        return self.angle_z

