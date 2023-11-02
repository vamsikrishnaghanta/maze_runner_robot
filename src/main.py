import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor control
motor_right_enable_pin = 27  # PWM pin for Motor 1 speed control
motor_right_pin1 = 5  # Motor 1 input pin 1
motor_right_pin2 = 22  # Motor 1 input pin 2

motor_left_enable_pin = 6  # PWM pin for Motor 2 speed control
motor_left_pin1 = 4  # Motor 2 input pin 1
motor_left_pin2 = 17  # Motor 2 input pin 2

# Define GPIO pins for motor encoders
encoder_right_c1 = 15  # GPIO pin for motor 1encoder c1
encoder_right_c2 = 14  # GPIO pin for motor 1 encoder c2
encoder_left_c1 = 21  # GPIO pin for motor 2 encoder c1
encoder_left_c2 = 20  # GPIO pin for motor 2 encoder c2

# Define GPIO pins for IR sensors
sensor_front = 26  # Change to your actual GPIO pin for front sensor
sensor_right = 19  # Change to your actual GPIO pin for right sensor
sensor_left = 13  # Change to your actual GPIO pin for left sensor

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_right_enable_pin, GPIO.OUT)
GPIO.setup(motor_right_pin1, GPIO.OUT)
GPIO.setup(motor_right_pin2, GPIO.OUT)

GPIO.setup(motor_left_enable_pin, GPIO.OUT)
GPIO.setup(motor_left_pin1, GPIO.OUT)
GPIO.setup(motor_left_pin2, GPIO.OUT)

GPIO.setup(encoder_right_c1, GPIO.IN)
GPIO.setup(encoder_right_c2, GPIO.IN)
GPIO.setup(encoder_left_c1, GPIO.IN)
GPIO.setup(encoder_left_c2, GPIO.IN)
GPIO.setup(sensor_front, GPIO.IN)
GPIO.setup(sensor_right, GPIO.IN)
GPIO.setup(sensor_left, GPIO.IN)

# Initialize PWM for motor speed control
motor1_pwm = GPIO.PWM(motor_right_enable_pin, 100)  # Frequency = 100 Hz
motor2_pwm = GPIO.PWM(motor_left_enable_pin, 100)

# Start PWM
motor1_pwm.start(0)  # Start with 0% duty cycle
motor2_pwm.start(0)


def check_sensors():
    front_sensor = GPIO.input(sensor_front)
    right_sensor = GPIO.input(sensor_right)
    left_sensor = GPIO.input(sensor_left)
    return front_sensor, right_sensor, left_sensor


class MotorControls:
    def __init__(self):

        # Define motor directions
        self.motor1_forward = True
        self.motor2_forward = True

        # Initialize PD controllers
        self.kp = 0.5
        self.ki = 0.00008
        self.kd = 0.01

        # For driving forward
        self.base_speed = 35
        self.max_speed = 80
        self.forward_speed_integral = 0
        self.forward_previous_speed_error = 0
        self.time_to_stop = 0.13  # time that robot should drive after detecting the wall

        # For rotation
        self.pulse_for_90_degree = 100  # ticks for 90 degrees rotation actually it 106 pulses
        self.pulse_for_180_degree = 190
        self.previous_angular_error = 0

        # Variables for encoder readings
        self.encoder_right_count_c1 = 0
        self.encoder_right_count_c2 = 0
        self.encoder_left_count_c1 = 0
        self.encoder_left_count_c2 = 0

        # Remove any previous event detection
        GPIO.remove_event_detect(encoder_right_c1)
        GPIO.remove_event_detect(encoder_right_c2)
        GPIO.remove_event_detect(encoder_left_c1)
        GPIO.remove_event_detect(encoder_left_c2)

        # Start event detection for all encoder channels
        self.event_detection()

    def clear_encoders(self):
        # Variables for encoder readings
        self.encoder_right_count_c1 = 0
        self.encoder_right_count_c2 = 0
        self.encoder_left_count_c1 = 0
        self.encoder_left_count_c2 = 0

    # Function to handle pulse detection for encoders
    def handle_pulse(self, encoder_index):
        if encoder_index == 1:
            self.encoder_right_count_c1 += 1
        elif encoder_index == 2:
            self.encoder_right_count_c2 += 1
        elif encoder_index == 3:
            self.encoder_left_count_c1 += 1
        elif encoder_index == 4:
            self.encoder_left_count_c2 += 1

    # Add event detection for all encoder channels
    def event_detection(self):
        GPIO.add_event_detect(encoder_right_c1, GPIO.RISING, callback=lambda x: self.handle_pulse(1))
        GPIO.add_event_detect(encoder_right_c2, GPIO.RISING, callback=lambda x: self.handle_pulse(2))
        GPIO.add_event_detect(encoder_left_c1, GPIO.RISING, callback=lambda x: self.handle_pulse(3))
        GPIO.add_event_detect(encoder_left_c2, GPIO.RISING, callback=lambda x: self.handle_pulse(4))

    # Function to control motors
    def set_motor_speeds(self, motor1_forward, motor2_forward, motor1_speed, motor2_speed):
        GPIO.output(motor_right_pin1, motor1_forward)
        GPIO.output(motor_right_pin2, not motor1_forward)
        GPIO.output(motor_left_pin1, motor2_forward)
        GPIO.output(motor_left_pin2, not motor2_forward)
        motor1_pwm.ChangeDutyCycle(motor1_speed)
        motor2_pwm.ChangeDutyCycle(motor2_speed)

    def stop_motors(self):
        self.set_motor_speeds(self.motor1_forward, self.motor2_forward, 0, 0)
        time.sleep(0.1)  # Delay for smoother control

    def make_straight(self):
        self.set_motor_speeds(False, False, self.base_speed, self.base_speed)
        print('Initiating make_straight method..!')
        time.sleep(1.5)
        self.stop_motors()

    # Function to drive the robot straight using a PID controller
    def drive_straight(self, target_pulses=1000, wait_time=0):
        # target_pulses are how much distance we want to drive the robot
        # wait_time is how much time we still want to drive the robot after detecting the wall
        self.clear_encoders()

        try:
            while True:
                front_sensor, right_sensor, _ = check_sensors()

                forward_speed_difference = self.encoder_right_count_c1 - self.encoder_left_count_c1
                # Define a synchronization PID controller
                forward_speed_error = forward_speed_difference
                self.forward_speed_integral += forward_speed_error
                forward_speed_derivative = forward_speed_error - self.forward_previous_speed_error
                self.forward_previous_speed_error = forward_speed_error

                # Calculate the control signal for synchronization
                pid_speed_control_signal = self.kp * forward_speed_error + self.ki * self.forward_speed_integral + self.kd * forward_speed_derivative

                #print(f'pid_speed_control:{pid_speed_control_signal} ')

                # motor speeds based on the synchronization control signal
                motor_right_speed = self.base_speed - pid_speed_control_signal
                motor_left_speed = self.base_speed + pid_speed_control_signal

                #print(f'motor_speed1:{motor_right_speed}, motor_speed2: {motor_left_speed}')

                # Limit motor speeds between 0 and 100
                motor_right_speed = max(0, min(self.max_speed, motor_right_speed))
                motor_left_speed = max(0, min(self.max_speed, motor_left_speed))

                # Set the motor speeds
                self.set_motor_speeds(True, True, motor_right_speed, motor_left_speed)

                avg_target_pulses = (self.encoder_right_count_c1 + self.encoder_left_count_c1)/2

                if avg_target_pulses >= target_pulses:
                    self.stop_motors()
                    break
                elif front_sensor == GPIO.LOW:
                    self.clear_encoders()
                    time.sleep(wait_time)
                    break

                time.sleep(0.01)  # A small delay to avoid busy waiting

        finally:
            # Stop the motors
            self.stop_motors()

    # Function to rotate the robot by 90 degrees towards east using a PD controller
    def rotate_in_degrees(self, direction, target_pulses_angular):
        self.clear_encoders()
        print('Turning pulses: ', target_pulses_angular)

        if direction == 'right':
            self.motor1_forward = False
        elif direction == 'left':
            self.motor1_forward = True
        else:
            print('Enter valid command for rotation')

        try:
            while self.encoder_right_count_c1 < target_pulses_angular and self.encoder_left_count_c1 < target_pulses_angular:
                #print('encoder:', self.encoder_right_count_c1, self.encoder_left_count_c1)

                angular_error = self.encoder_right_count_c1 - self.encoder_left_count_c1
                angular_derivative = angular_error - self.previous_angular_error
                self.previous_angular_error = angular_error

                # Calculate control signals for both motors
                angular_control_signal = self.kp * angular_error + self.kd * angular_derivative

                # Calculate motor speeds based on control signals
                motor_right_speed = self.base_speed - angular_control_signal
                motor_left_speed = self.base_speed + angular_control_signal

                # Limit motor speeds between 0 and 100
                motor1_speed = max(0, min(self.max_speed, motor_right_speed))
                motor2_speed = max(0, min(self.max_speed, motor_left_speed))

                # Set motor directions based on turn direction
                self.set_motor_speeds(self.motor1_forward, not self.motor1_forward, motor1_speed, motor2_speed)

                time.sleep(0.01)  # A small delay to avoid busy waiting

        finally:
            # Stop the motors
            self.stop_motors()


# Create an instance of the MazeSolver class
motor_controls = MotorControls()


# Function to handle cases
def solve_maze():
    print('Started solving maze')
    try:
        while True:
            front_sensor, right_sensor, left_sensor = check_sensors()

            if front_sensor == GPIO.HIGH and right_sensor == GPIO.LOW:  # No wall in front
                print('Case 1: Front sensor not detected')
                motor_controls.drive_straight(wait_time=motor_controls.time_to_stop)
                time.sleep(3)

            elif front_sensor == GPIO.LOW and right_sensor == GPIO.LOW and left_sensor == GPIO.HIGH:
                # Both right wall and front wall are detected 'corner'
                print('Case 2: Front and right sensors detected - corner')
                motor_controls.rotate_in_degrees('left', motor_controls.pulse_for_90_degree)
                time.sleep(3)
                motor_controls.make_straight()

            elif front_sensor == GPIO.LOW and right_sensor == GPIO.LOW and left_sensor == GPIO.LOW:
                # Case 3: Dead-end, rotate 180 degrees
                print('Case 3: All three front, left and right sensors detected - DEAD END')
                motor_controls.rotate_in_degrees('right', motor_controls.pulse_for_180_degree)
                time.sleep(3)
                motor_controls.make_straight()

            elif right_sensor == GPIO.HIGH:  # No wall on the right
                print('Case 4: right sensor not detected')
                print('Driving 50 pulse straight....')
                motor_controls.stop_motors()
                time.sleep(1)
                motor_controls.drive_straight(target_pulses=50)
                time.sleep(1)
                print('Turning right....')
                motor_controls.rotate_in_degrees('right', motor_controls.pulse_for_90_degree)
                time.sleep(1)
                motor_controls.make_straight()
                motor_controls.drive_straight(wait_time=motor_controls.time_to_stop)
                if right_sensor == GPIO.HIGH and left_sensor == GPIO.HIGH:
                    print('Turning right...')
                    motor_controls.rotate_in_degrees('right', motor_controls.pulse_for_90_degree)
                    motor_controls.drive_straight()
                    time.sleep(1)

            elif front_sensor == GPIO.HIGH and right_sensor == GPIO.HIGH and left_sensor == GPIO.HIGH:
                print('case 5: No walls detected, driving straight...')
                motor_controls.drive_straight()

    finally:
        # Stop the motors
        motor_controls.stop_motors()


# Start solving the maze
solve_maze()

# Clean up GPIO settings
GPIO.cleanup()
