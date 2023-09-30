import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor control
motor1_input1 = 4   # Change to your actual GPIO pin
motor1_input2 = 14  # Change to your actual GPIO pin
motor2_input1 = 17  # Change to your actual GPIO pin
motor2_input2 = 18  # Change to your actual GPIO pin
motor1_pwm = 22  # enable1
motor2_pwm = 23  # enable2

# Define GPIO pins for motor encoders
motor1_encoder_c1 = 24  # Change to your actual GPIO pin for motor 1 encoder c1
motor1_encoder_c2 = 10  # Change to your actual GPIO pin for motor 1 encoder c2
motor2_encoder_c1 = 9   # Change to your actual GPIO pin for motor 2 encoder c1
motor2_encoder_c2 = 25  # Change to your actual GPIO pin for motor 2 encoder c2

# Define GPIO pins for IR sensors
sensor_front = 8  # Change to your actual GPIO pin for front sensor
sensor_right = 7  # Change to your actual GPIO pin for right sensor
sensor_left = 5   # Change to your actual GPIO pin for left sensor

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_input1, GPIO.OUT)
GPIO.setup(motor1_input2, GPIO.OUT)
GPIO.setup(motor2_input1, GPIO.OUT)
GPIO.setup(motor2_input2, GPIO.OUT)
GPIO.setup(motor1_pwm, GPIO.OUT)
GPIO.setup(motor2_pwm, GPIO.OUT)
GPIO.setup(motor1_encoder_c1, GPIO.IN)
GPIO.setup(motor1_encoder_c2, GPIO.IN)
GPIO.setup(motor2_encoder_c1, GPIO.IN)
GPIO.setup(motor2_encoder_c2, GPIO.IN)
GPIO.setup(sensor_front, GPIO.IN)
GPIO.setup(sensor_right, GPIO.IN)
GPIO.setup(sensor_left, GPIO.IN)

# Initialize PWM for motor speed control
motor1_pwm = GPIO.PWM(motor1_pwm, 100)  # Frequency = 100 Hz
motor2_pwm = GPIO.PWM(motor2_pwm, 100)

# Start PWM
motor1_pwm.start(0)  # Start with 0% duty cycle
motor2_pwm.start(0)

# Define motor directions
motor1_forward = True
motor2_forward = True

# Function to control motors
def set_motor_speeds(motor1_forward, motor2_forward, motor1_speed, motor2_speed):
    GPIO.output(motor1_input1, motor1_forward)
    GPIO.output(motor1_input2, not motor1_forward)
    GPIO.output(motor2_input1, motor2_forward)
    GPIO.output(motor2_input2, not motor2_forward)
    motor1_pwm.ChangeDutyCycle(motor1_speed)
    motor2_pwm.ChangeDutyCycle(motor2_speed)

class MazeSolver:
    def __init__(self):
        # Initialize GPIO inputs for motor control
        self.motor1_input1 = motor1_input1
        self.motor1_input2 = motor1_input2
        self.motor2_input1 = motor2_input1
        self.motor2_input2 = motor2_input2

        # Initialize motor encoder values
        self.prev_encoder1 = 0
        self.prev_encoder2 = 0

        # Initialize PD controllers
        self.kp_distance = 0.1  # Adjust these values as needed
        self.kd_distance = 0.01
        self.kp_angular = 0.1
        self.kd_angular = 0.01

    def read_encoder_values(self):
        motor1_encoder1 = GPIO.input(motor1_encoder_c1)
        motor1_encoder2 = GPIO.input(motor1_encoder_c2)
        motor2_encoder1 = GPIO.input(motor2_encoder_c1)
        motor2_encoder2 = GPIO.input(motor2_encoder_c2)
        return motor1_encoder1, motor1_encoder2, motor2_encoder1, motor2_encoder2

    def read_encoder(self, encoder_c1, encoder_c2):
        return GPIO.input(encoder_c1) ^ GPIO.input(encoder_c2)

    def calculate_distance_error(self, target_distance):
        encoder1, _, encoder2, _ = self.read_encoder_values()
        distance_travelled = (encoder1 + encoder2) / 2
        distance_error = target_distance - distance_travelled
        return distance_error

    def calculate_angular_error(self, target_angle):
        encoder1, _, encoder2, _ = self.read_encoder_values()
        angle_rotated = (encoder2 - encoder1)  # Assuming positive angle is counterclockwise
        angular_error = target_angle - angle_rotated
        return angular_error

    # Function to drive the robot straight using a PD controller
    def drive_straight_with_pd(self):
        previous_error = 0

        while True:
            if self.read_encoder(motor1_encoder_c1, motor1_encoder_c2) or \
                    self.read_encoder(motor2_encoder_c1, motor2_encoder_c2):
                ticks_motor1 = self.read_encoder(motor1_encoder_c1, motor1_encoder_c2)
                ticks_motor2 = self.read_encoder(motor2_encoder_c1, motor2_encoder_c2)

                # Calculate speed difference (error) between the two wheels
                error = ticks_motor1 - ticks_motor2

                derivative = error - previous_error
                control_signal = self.kp_distance * error + self.kd_distance * derivative

                control_signal = max(-50, min(50, control_signal))

                # Adjust motor speeds using the control signal
                motor1_speed = 50 + control_signal
                motor2_speed = 50 - control_signal
                set_motor_speeds(True, True, motor1_speed, motor2_speed)

                previous_error = error

            else:
                set_motor_speeds(True, True, 0, 0)

    def move_forward(self, target_distance):
        while True:
            distance_error = self.calculate_distance_error(target_distance)
            angular_error = self.calculate_angular_error(0)  # Maintain zero angular error
            speed_correction = self.kp_distance * distance_error + self.kd_distance * (distance_error - self.prev_distance_error)
            angular_correction = self.kp_angular * angular_error + self.kd_angular * (angular_error - self.prev_angular_error)

            motor_speed = 100 + speed_correction  # Adjust the base motor speed as needed
            left_speed = motor_speed - angular_correction
            right_speed = motor_speed + angular_correction

            self.set_motor_speed(left_speed, right_speed)

            self.prev_distance_error = distance_error
            self.prev_angular_error = angular_error

            time.sleep(0.1)  # Delay for smoother control

    def rotate_clockwise(self, target_angle):
        while True:
            angular_error = self.calculate_angular_error(target_angle)
            angular_correction = self.kp_angular * angular_error + self.kd_angular * (angular_error - self.prev_angular_error)

            motor_speed = 50 + angular_correction  # Adjust the base motor speed as needed
            left_speed = -motor_speed
            right_speed = motor_speed

            self.set_motor_speed(left_speed, right_speed)

            self.prev_angular_error = angular_error

            time.sleep(0.1)  # Delay for smoother control

    def rotate_counterclockwise(self, target_angle):
        while True:
            angular_error = self.calculate_angular_error(-target_angle)  # Negative target angle for counterclockwise rotation
            angular_correction = self.kp_angular * angular_error + self.kd_angular * (angular_error - self.prev_angular_error)

            motor_speed = 50 + angular_correction  # Adjust the base motor speed as needed
            left_speed = motor_speed
            right_speed = -motor_speed

            self.set_motor_speed(left_speed, right_speed)

            self.prev_angular_error = angular_error

            time.sleep(0.1)  # Delay for smoother control

    def set_motor_speed(self, left_speed, right_speed):
        # Implement motor speed control logic here
        # Set the motor speeds based on input values
        pass

    def stop_motors(self):
        self.set_motor_speed(0, 0)

# Create an instance of the MazeSolver class
maze_solver = MazeSolver()

# Example usage:
# maze_solver.move_forward(200)  # Move forward 200 units
# maze_solver.rotate_clockwise(90)  # Rotate clockwise by 90 degrees
# maze_solver.rotate_counterclockwise(45)  # Rotate counterclockwise by 45 degrees

# Clean up GPIO settings
GPIO.cleanup()



##########################################################################################################

# forward
import RPi.GPIO as GPIO
import time
import csv


# Define the motor control pins
motor1_enable_pin = 22  # PWM pin for Motor 1 speed control
motor1_pin1 = 4  # Motor 1 input pin 1
motor1_pin2 = 14  # Motor 1 input pin 2

motor2_enable_pin = 23  # PWM pin for Motor 2 speed control
motor2_pin1 = 17  # Motor 2 input pin 1
motor2_pin2 = 18  # Motor 2 input pin 2

# Define encoder pins (c1 and c2) for both motors
encoder1_c1 = 24
encoder1_c2 = 10
encoder2_c1 = 26
encoder2_c2 = 20

# Create empty lists to store the data
control_signal1_data = []
control_signal2_data = []
pid_speed_control_data = []
motor_speed1_data = []
motor_speed2_data = []
encoder1 = []
encoder2 = []

# Initialize GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_enable_pin, GPIO.OUT)
GPIO.setup(motor1_pin1, GPIO.OUT)
GPIO.setup(motor1_pin2, GPIO.OUT)

GPIO.setup(motor2_enable_pin, GPIO.OUT)
GPIO.setup(motor2_pin1, GPIO.OUT)
GPIO.setup(motor2_pin2, GPIO.OUT)

GPIO.setup(encoder1_c1, GPIO.IN)
GPIO.setup(encoder1_c2, GPIO.IN)
GPIO.setup(encoder2_c1, GPIO.IN)
GPIO.setup(encoder2_c2, GPIO.IN)


# Initialize PWM for motor speed control
motor1_pwm = GPIO.PWM(motor1_enable_pin, 100)  # Frequency = 100 Hz
motor2_pwm = GPIO.PWM(motor2_enable_pin, 100)


# Start PWM
motor1_pwm.start(0)  # Start with 0% duty cycle
motor2_pwm.start(0)

# Function to set motor speeds
def set_motor_speeds(speed1, speed2):
    motor1_pwm.ChangeDutyCycle(speed1)
    motor2_pwm.ChangeDutyCycle(speed2)


# Variables for PD controller
target_pulses = 500  # Adjust as needed
kp = 0.2  # Proportional gain
ki = 0.08  # Integral gain
kd = 0.1  # Derivative gain

speed_integral = 0
previous_speed_error = 0
previous_error1 = 0
previous_error2 = 0

# Variables for encoder readings
encoder_right_count_c1 = 0
encoder_right_count_c2 = 0
encoder_left_count_c1 = 0
encoder_left_count_c2 = 0


# Function to handle pulse detection for c1 encoders
# Function to handle pulse detection for encoders
def handle_pulse(encoder_index):
    global encoder_right_count_c1, encoder_right_count_c2, encoder_left_count_c1, encoder_left_count_c2

    if encoder_index == 1:
        pulse_count11 += 1
    elif encoder_index == 2:
        pulse_count12 += 1
    elif encoder_index == 3:
        pulse_count21 += 1
    elif encoder_index == 4:
        pulse_count22 += 1


# Add event detection for all encoder channels
GPIO.add_event_detect(encoder1_c1, GPIO.RISING, callback=lambda x: handle_pulse(1))
GPIO.add_event_detect(encoder1_c2, GPIO.RISING, callback=lambda x: handle_pulse(2))
GPIO.add_event_detect(encoder2_c1, GPIO.RISING, callback=lambda x: handle_pulse(3))
GPIO.add_event_detect(encoder2_c2, GPIO.RISING, callback=lambda x: handle_pulse(4))

try:
    while True:
        current_pulses1 = encoder_right_count_c1
        current_pulses2 = encoder_left_count_c1
        error1 = target_pulses - current_pulses1
        error2 = target_pulses - current_pulses2
        derivative1 = error1 - previous_error1
        derivative2 = error2 - previous_error2

        # Calculate control signals for both motors
        control_signal1 = kp * error1 + kd * derivative1
        control_signal2 = kp * error2 + kd * derivative2

        print(f'control_signal1:{control_signal1}, control_signal2: {control_signal2}')

        # Calculate motor speeds based on control signals
        motor_speed1 = 20 + control_signal1
        motor_speed2 = 20 + control_signal2

        # Calculate the difference in speeds between the two motors
        speed_difference = motor_speed1 - motor_speed2
        # Define a synchronization PID controller
        speed_error = speed_difference
        speed_integral += speed_error
        speed_derivative = speed_error - previous_speed_error

        # Calculate the control signal for synchronization
        pid_speed_control_signal = kp * speed_error + ki * speed_integral + kd * speed_derivative

        print(f'pid_speed_control:{pid_speed_control_signal} ')

        # Adjust the motor speeds based on the synchronization control signal
        motor_speed1 = motor_speed1 - pid_speed_control_signal
        motor_speed2 = motor_speed2 + pid_speed_control_signal

        print(f'motor_speed1:{motor_speed1}, motor_speed2: {motor_speed2}')

        # Limit motor speeds between 0 and 100
        motor_speed1 = max(0, min(100, motor_speed1))
        motor_speed2 = max(0, min(100, motor_speed2))

        # Set motor directions for forward movement
        GPIO.output(motor1_pin1, GPIO.HIGH)
        GPIO.output(motor1_pin2, GPIO.LOW)
        GPIO.output(motor2_pin1, GPIO.HIGH)
        GPIO.output(motor2_pin2, GPIO.LOW)

        # Set the motor speeds
        set_motor_speeds(motor_speed1, motor_speed2)

        previous_error1 = error1
        previous_error2 = error2
        previous_speed_error = speed_error

        control_signal1_data.append(control_signal1)
        control_signal2_data.append(control_signal2)
        pid_speed_control_data.append(pid_speed_control_signal)
        motor_speed1_data.append(motor_speed1)
        motor_speed2_data.append(motor_speed2)
        encoder1.append(encoder_right_count_c1)
        encoder2.append(encoder_left_count_c1)

        avg_pulses = (encoder_right_count_c1 + encoder_left_count_c1) / 2
        if avg_pulses >= target_pulses:
            break

        time.sleep(0.01)  # A small delay to avoid busy waiting


finally:
    # Stop the motors
    set_motor_speeds(0, 0)

    # Clean up GPIO and remove event detection
    GPIO.remove_event_detect(encoder1_c1)
    GPIO.remove_event_detect(encoder1_c2)
    GPIO.remove_event_detect(encoder2_c1)
    GPIO.remove_event_detect(encoder2_c2)
    GPIO.cleanup()

    # Print the number of pulses detected for all encoders
    print(f"Motor 1 (c1): Detected {encoder_right_count_c1} pulses for one full rotation.")
    print(f"Motor 1 (c2): Detected {encoder_right_count_c2} pulses for one full rotation.")
    print(f"Motor 2 (c1): Detected {encoder_left_count_c1} pulses for one full rotation.")
    print(f"Motor 2 (c2): Detected {encoder_left_count_c2} pulses for one full rotation.")

    # Save data to a CSV file
    data = {
        'Encoder 1': encoder1,
        'Encoder 2': encoder2,
        'Control Signal 1': control_signal1_data,
        'Control Signal 2': control_signal2_data,
        'PID Speed Control': pid_speed_control_data,
        'Motor Speed 1': motor_speed1_data,
        'Motor Speed 2': motor_speed2_data,

    }

    csv_file_name = 'motor_data.csv'  # Choose a suitable file name

    try:
        with open(csv_file_name, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=data.keys())
            writer.writeheader()
            for i in range(len(control_signal1_data)):
                row = {key: data[key][i] for key in data.keys()}
                writer.writerow(row)

        print(f'Data saved to {csv_file_name}')
    except Exception as e:
        print(f'Error saving data to CSV: {e}')

