import RPi.GPIO as GPIO
import time
import csv

# Define the motor control pins
motor_right_enable_pin = 22  # PWM pin for Motor 1 speed control
motor_right_pin1 = 4  # Motor 1 input pin 1
motor_right_pin2 = 14  # Motor 1 input pin 2

motor_left_enable_pin = 23  # PWM pin for Motor 2 speed control
motor_left_pin1 = 17  # Motor 2 input pin 1
motor_left_pin2 = 18  # Motor 2 input pin 2

# Define encoder pins (c1 and c2) for both motors
encoder_right_c1 = 26
encoder_right_c2 = 20
encoder_left_c1 = 24
encoder_left_c2 = 10

# Create empty lists to store the data
pid_speed_control_data = []
speed_difference_data = []
motor_speed1_data = []
motor_speed2_data = []
right_encoder1_data = []
right_encoder2_data = []
left_encoder1_data = []
left_encoder2_data = []

# Initialize GPIO pins
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

# Initialize PWM for motor speed control
motor1_pwm = GPIO.PWM(motor_right_enable_pin, 100)  # Frequency = 100 Hz
motor2_pwm = GPIO.PWM(motor_left_enable_pin, 100)

# Start PWM
motor1_pwm.start(0)  # Start with 0% duty cycle
motor2_pwm.start(0)


# Function to set motor speeds
def set_motor_speeds(speed1, speed2):
    motor1_pwm.ChangeDutyCycle(speed1)
    motor2_pwm.ChangeDutyCycle(speed2)


# Variables for encoder readings
encoder_right_count_c1 = 0
encoder_right_count_c2 = 0
encoder_left_count_c1 = 0
encoder_left_count_c2 = 0


# Function to handle pulse detection for encoders
def handle_pulse(encoder_index):
    global encoder_right_count_c1, encoder_right_count_c2, encoder_left_count_c1, encoder_left_count_c2

    if encoder_index == 1:
        encoder_right_count_c1 += 1
    elif encoder_index == 2:
        encoder_right_count_c2 += 1
    elif encoder_index == 3:
        encoder_left_count_c1 += 1
    elif encoder_index == 4:
        encoder_left_count_c2 += 1


# Add event detection for all encoder channels
GPIO.add_event_detect(encoder_right_c1, GPIO.RISING, callback=lambda x: handle_pulse(1))
GPIO.add_event_detect(encoder_right_c2, GPIO.RISING, callback=lambda x: handle_pulse(2))
GPIO.add_event_detect(encoder_left_c1, GPIO.RISING, callback=lambda x: handle_pulse(3))
GPIO.add_event_detect(encoder_left_c2, GPIO.RISING, callback=lambda x: handle_pulse(4))


# Variables for PD controller
base_speed = 15
max_speed = 40
target_pulses = 1000  # Adjust as needed
kp = 0.001  # Proportional gain
ki = 0.0001  # Integral gain
kd = 0.1  # Derivative gain
current_time = time.time()

forward_speed_integral = 0
forward_previous_speed_error = 0
motor_right_speed = 0
motor_left_speed = 0
forward_speed_difference = 0



try:
    while True:
        # Calculate the difference in speeds between the two motors
        # motor_speed1 = 5 + pulse_count11/current_time
        # motor_speed2 = 5 + pulse_count21/current_time
        forward_speed_difference = encoder_right_count_c1 - encoder_left_count_c1

        # Define a synchronization PID controller
        forward_speed_error = forward_speed_difference
        forward_speed_integral += forward_speed_error
        forward_speed_derivative = forward_speed_error - forward_previous_speed_error
        forward_previous_speed_error = forward_speed_error

        # Calculate the control signal for synchronization
        pid_speed_control_signal = kp * forward_speed_error + ki * forward_speed_integral + kd * forward_speed_derivative

        print(f'pid_speed_control:{pid_speed_control_signal} ')

        # motor speeds based on the synchronization control signal
        motor_right_speed = base_speed - pid_speed_control_signal
        motor_left_speed = base_speed + pid_speed_control_signal

        print(f'motor_speed1:{motor_right_speed}, motor_speed2: {motor_left_speed}')

        # Limit motor speeds between 0 and 100
        motor_right_speed = max(0, min(max_speed, motor_right_speed))
        motor_left_speed = max(0, min(max_speed, motor_left_speed))

        # Set motor directions for forward movement
        GPIO.output(motor_right_pin1, GPIO.HIGH)
        GPIO.output(motor_right_pin2, GPIO.LOW)
        GPIO.output(motor_left_pin1, GPIO.HIGH)
        GPIO.output(motor_left_pin2, GPIO.LOW)

        # Set the motor speeds
        set_motor_speeds(motor_right_speed, motor_left_speed)

        pid_speed_control_data.append(pid_speed_control_signal)
        speed_difference_data.append(forward_speed_difference)
        motor_speed1_data.append(motor_right_speed)
        motor_speed2_data.append(motor_left_speed)
        right_encoder1_data.append(encoder_right_count_c1)
        right_encoder2_data.append(encoder_right_count_c2)
        left_encoder1_data.append(encoder_left_count_c1)
        left_encoder2_data.append(encoder_left_count_c2)

        if encoder_right_count_c1 >= target_pulses or encoder_left_count_c1 >= target_pulses:
            break

        time.sleep(0.01)  # A small delay to avoid busy waiting


finally:
    # Stop the motors
    set_motor_speeds(0, 0)

    # Clean up GPIO and remove event detection
    GPIO.remove_event_detect(encoder_right_c1)
    GPIO.remove_event_detect(encoder_right_c2)
    GPIO.remove_event_detect(encoder_left_c1)
    GPIO.remove_event_detect(encoder_left_c2)
    GPIO.cleanup()

    # Print the number of pulses detected for all encoders
    print(f"Motor 1 (c1): Detected {encoder_right_count_c1} pulses for one full rotation.")
    print(f"Motor 1 (c2): Detected {encoder_right_count_c2} pulses for one full rotation.")
    print(f"Motor 2 (c1): Detected {encoder_left_count_c1} pulses for one full rotation.")
    print(f"Motor 2 (c2): Detected {encoder_left_count_c2} pulses for one full rotation.")

    # Save data to a CSV file
    data = {
        'Right Encoder 1': right_encoder1_data,
        'Right Encoder 2': right_encoder2_data,
        'Left Encoder 1': left_encoder1_data,
        'Left Encoder 2': left_encoder2_data,
        'PID Speed Control': pid_speed_control_data,
        'Speed Difference': speed_difference_data,
        'Motor Speed 1': motor_speed1_data,
        'Motor Speed 2': motor_speed2_data,
    }

    csv_file_name = f'motor_data_forward_{target_pulses}pulses_kp{kp}_ki{ki}_kd{kd}.csv'

    try:
        with open(csv_file_name, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=list(data.keys()))
            writer.writeheader()
            for i in range(len(right_encoder1_data)):
                row = {key: data[key][i] for key in data.keys()}
                writer.writerow(row)

        print(f'Data saved to {csv_file_name}')
    except Exception as e:
        print(f'Error saving data to CSV: {e}')

