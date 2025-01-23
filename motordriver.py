from machine import Pin, PWM
import time 
import math
from ili9341 import color565  # Import color565 for color handling
from xglcd_font import XglcdFont
import random


class motordriver:
    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size, display, font):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.encoder1_irq_handler)
        self.encoder2 = Pin(encoder2_pin, Pin.IN)

        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm1 = PWM(self.in1)
        self.pwm2 = PWM(self.in2)
        self.pwm1.freq(1000)
        self.pwm2.freq(1000)
        self.wheel_size = wheel_size
        self.degrees = 0
        
        self.font = font  # Store the font for visualization
        self.display = display  # Store the display object

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self.integral_flag = False
        
        self.plot_step = 1  # Plot update step: Update plot every 'plot_step' iterations
        self.total_iterations = 0  # Track total number of PID iterations
        self.iteration = 0  # Initialize iteration counter for plotting
        self.iteration_counter = 0  # To control how often pixels are plotted

        self.graph_x = 0  # Start drawing from the leftmost pixel
        
        
    def encoder1_irq_handler(self, pin):
        encoder1_state = self.encoder1.value()
        encoder2_state = self.encoder2.value()
        
        if encoder1_state == encoder2_state:
            self.degrees += 1
        else:
            self.degrees -= 1

        #print("Degrees: ", self.degrees)

    def motgo(self, speed):
        pwm_value = int(min(max(abs(speed), 0), 100) * 10.23)  # Map -100 to 100 to 0 to 1023

        if speed > 0:
            # Forward direction
            self.pwm1.duty(pwm_value)
            self.pwm2.duty(0)
        elif speed < 0:
            # Reverse direction
            self.pwm1.duty(0)
            self.pwm2.duty(pwm_value)
        else:
            # Stop the motor
            self.pwm1.duty(0)
            self.pwm2.duty(0)    
            self.in1.value(0)
            self.in2.value(0)  # Explicitly stop the motor  
    
    def PIDcalc(self, inp, sp, kp, ki, kd, color):
        current_time = time.ticks_ms()
        elapsed_time = (current_time - self.previous_time) / 1000.0

        # Ensure elapsed_time is positive to avoid division by zero
        if elapsed_time <= 0:
            return 0

        # Calculate error
        error = sp - inp

        # Reset integral term if error changes direction
        if error * self.last_error < 0:
            self.integral_flag = True
            self.cum_error = 0
        else:
            self.integral_flag = False

        # Update integral only if not flagged
        if not self.integral_flag:
            self.cum_error += error * elapsed_time

        # Calculate derivative and output
        rate_error = (error - self.last_error) / elapsed_time
        out = kp * error + ki * self.cum_error + kd * rate_error

        # Save current error and time for next iteration
        self.last_error = error
        self.previous_time = current_time

        # Clamp output to motor limits
        out = max(-254, min(254, out))

        # Increment the iteration counter for PID
        self.iteration_counter += 1
        if self.iteration_counter >= self.plot_step:
            time.sleep(0.01)
            self.plot_pid(out, color)
            self.iteration_counter = 0  # Reset counter

        # Return PID output
        return out


    
    
    def plot_pid(self, error, color):
        # Initialize plot iteration counter if not already defined
        if not hasattr(self, 'plot_iteration'):
            self.plot_iteration = 0  # Initialize plot counter

        # Screen dimensions
        screen_height = 240
        screen_width = 240

        # Scale the error to fit the screen height
        max_error = 254
        min_error = -254
        y_position = int(((error - min_error) / (max_error - min_error)) * screen_height)
        y_position = max(0, min(screen_height - 1, screen_height - y_position))

        # Adjust x-coordinate based on overall PID loop iteration
        x_position = (self.total_iterations // self.plot_step) % screen_width

        # Draw the pixel
        self.display.draw_pixel(x_position, y_position, color)

        # Increment counters
        self.plot_iteration += 1
        self.total_iterations += 1  # Increment total iterations




    def display_pid_values(self, kp, ki, kd, color, line_index):
        # Line index determines where to display the PID values on the screen
        text_y = 210 + (line_index * 28)  # Adjust vertical spacing based on font size

        # Format the text
        text = f"Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f}"
        
        # Draw thicker text by offsetting slightly in multiple directions
        offsets = [(0, 0), (1, 0), (0, 1)]  # Fewer offsets for reduced overhead
   
        # Display the text in the specified color using the provided font
        for dx, dy in offsets:        
            self.display.draw_text(5, text_y, text, self.font, color)


   

    def reset_graph(self):
        """Reset graph data after comparison."""
        self.previous_graph_data = self.graph_data  # Store the current graph for future comparison
        self.graph_data = []  # Clear the current graph data
        self.graph_x = 0  # Reset x position for the new run


    def reset_iterations(self):
        self.iteration = 0
        self.iteration_counter = 0



    def godegrees(self, angle, times):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, 1, 1, 0)
            motspeed = max(-254, min(254, motspeed))  # Clamp the speed to [-254, 254]
            self.motgo(motspeed)

    
    def godegreesp(self, angle, times, kp, ki, kd, color, line_index):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, kp, ki, kd, color)
            motspeed = max(-254, min(254, motspeed))
            self.motgo(motspeed)

        self.display_pid_values(kp, ki, kd, color, line_index)
        print('reached ', self.degrees, 'degrees')
        
        
        
    def gomm(self, distance, times):
        deg = (distance / (self.wheel_size * math.pi)) * 360
        self.godegrees(deg, times)
        dist_covered = (self.degrees * self.wheel_size * math.pi) / 360.0
        return dist_covered

    def gommp(self, distance, times, kp, ki, kd):
        deg = (distance / (self.wheel_size * math.pi)) * 360
        self.godegreesp(deg, times, kp, ki, kd)
        dist_covered = (self.degrees * self.wheel_size * math.pi) / 360.0

        return dist_covered
    
    
