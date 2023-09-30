import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor control
motor1_input1 = 4   # Change to your actual GPIO pin
motor1_input2 = 14  # Change to your actual GPIO pin
motor2_input1 = 17  # Change to your actual GPIO pin
motor2_input2 = 18  # Change to your actual GPIO pin
motor1_pwm_pin = 22  # enable1
motor2_pwm_pin = 23  # enable2

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_input1, GPIO.OUT)
GPIO.setup(motor1_input2, GPIO.OUT)
GPIO.setup(motor2_input1, GPIO.OUT)
GPIO.setup(motor2_input2, GPIO.OUT)
GPIO.setup(motor1_pwm_pin, GPIO.OUT)
GPIO.setup(motor2_pwm_pin, GPIO.OUT)

# Initialize PWM for motor speed control
motor1_pwm = GPIO.PWM(motor1_pwm_pin, 100)  # Frequency = 100 Hz
motor2_pwm = GPIO.PWM(motor2_pwm_pin, 100)

# Start PWM
motor1_pwm.start(0)  # Start with 0% duty cycle
motor2_pwm.start(0)

class MotorControls:
    def __init__(self):
        self.motor1_input1 = motor1_input1
        self.motor1_input2 = motor1_input2
        self.motor2_input1 = motor2_input1
        self.motor2_input2 = motor2_input2

    def set_motor_speeds(self, motor1_forward, motor2_forward, motor1_speed, motor2_speed):
        GPIO.output(self.motor1_input1, motor1_forward)
        GPIO.output(self.motor1_input2, not motor1_forward)
        GPIO.output(self.motor2_input1, motor2_forward)
        GPIO.output(self.motor2_input2, not motor2_forward)
        motor1_pwm.ChangeDutyCycle(motor1_speed)
        motor2_pwm.ChangeDutyCycle(motor2_speed)

    def move_forward(self, distance):
        ticks_per_cm = 10  # Adjust this value based on motor encoder and wheel size
        target_ticks = distance * ticks_per_cm

        while True:
            # Implement PD controller logic for distance control
            # Calculate error, derivative, and control signal

            # Adjust motor speeds using the control signal
            self.set_motor_speeds(True, True, motor1_speed, motor2_speed)

            # Check if target_ticks are reached and break the loop if true
            if avg_ticks >= target_ticks:
                break

            # Implement a delay for smoother control
            time.sleep(0.01)

        # Stop motors after reaching the target distance
        self.stop_motors()

    def rotate_degrees(self, direction, angle):
        ticks_per_degree = 10  # Adjust this value based on motor encoder and wheel size
        target_ticks = angle * ticks_per_degree

        # Set motor directions based on the rotation direction
        if direction == 'right':
            motor1_forward = True
            motor2_forward = False
        elif direction == 'left':
            motor1_forward = False
            motor2_forward = True
        else:
            print('Invalid rotation direction')
            return

        while True:
            # Implement PD controller logic for angular control
            # Calculate error, derivative, and control signal

            # Adjust motor speeds using the control signal
            self.set_motor_speeds(motor1_forward, motor2_forward, motor1_speed, motor2_speed)

            # Check if target_ticks are reached and break the loop if true
            if avg_ticks >= target_ticks:
                break

            # Implement a delay for smoother control
            time.sleep(0.01)

        # Stop motors after completing the rotation
        self.stop_motors()

    def stop_motors(self):
        self.set_motor_speeds(True, True, 0, 0)
        time.sleep(0.1)  # Delay for smoother control

# Create an instance of the MotorControls class
motor_controls = MotorControls()

# Example usage:
motor_controls.rotate_degrees('right', 90)  # Rotate clockwise by 90 degrees
motor_controls.move_forward(20)  # Move forward 20 cm
motor_controls.rotate_degrees('left', 45)  # Rotate counterclockwise by 45 degrees

# Clean up GPIO settings
GPIO.cleanup()
