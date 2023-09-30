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
pid_speed_control_right_data = []
pid_speed_control_left_data = []
motor_speed1_data = []
motor_speed2_data = []
encoder1_data = []
encoder2_data = []

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
speed = 20
target_pulses = 10000  # Adjust as needed
kp = 0.001  # Proportional gain
ki = 0.001  # Integral gain
kd = 0.1  # Derivative gain

# Variables for encoder readings
encoder_right_count_c1 = 0
encoder_right_count_c2 = 0
encoder_left_count_c1 = 0
encoder_left_count_c2 = 0

left_last_counter = 0
right_last_counter = 0
start_time = time.time()

# Error variables
speed_integral_right = 0
previous_speed_error_right = 0
speed_integral_left = 0
previous_speed_error_left = 0
pid_speed_control_signal_right = 0
pid_speed_control_signal_left = 0
motor_speed1 = 0
motor_speed2 = 0


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
        # Read encoder values
        current_pulse_count11 = encoder_right_count_c1
        current_pulse_count21 = encoder_left_count_c1

        # # Integration and PID control for synchronization
        # left_update_value = (left_update_value << 2) | (left_A.value() << 1) | left_B.value() & 0b00001111
        # left_counter = outcome(left_update_value)
        #
        # right_update_value = (right_update_value << 2) | (right_A.value() << 1) | right_B.value() & 0b00001111
        # right_counter = outcome(right_update_value)

        current_time = time.time()
        delta_time = current_time - start_time

        if delta_time > 0.1:  # Change to the desired time interval (e.g., 0.1 seconds)
            left_delta_speed = (encoder_left_count_c1 - left_last_counter) * 10
            right_delta_speed = (encoder_right_count_c1 - right_last_counter) * 10

            start_time = current_time

            left_last_counter = encoder_left_count_c1
            right_last_counter = encoder_right_count_c1

            left_error = speed - left_delta_speed
            speed_integral_left += left_error

            pid_speed_control_signal_left = int(
                left_error * kp + speed_integral_left * ki + (left_error - previous_speed_error_left) * kd
            )

            right_error = speed - right_delta_speed
            speed_integral_right += right_error

            pid_speed_control_signal_right = int(
                right_error * kp + speed_integral_right * ki + (right_error - previous_speed_error_right) * kd
            )

            # Apply PID control to motor speeds
            motor_speed1 = speed + pid_speed_control_signal_right
            motor_speed2 = speed + pid_speed_control_signal_left

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

        pid_speed_control_right_data.append(pid_speed_control_signal_right)
        pid_speed_control_left_data.append(pid_speed_control_signal_left)
        motor_speed1_data.append(motor_speed1)
        motor_speed2_data.append(motor_speed2)
        encoder1_data.append(encoder_right_count_c1)
        encoder2_data.append(encoder_left_count_c1)

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
        'Encoder 1': encoder1_data,
        'Encoder 2': encoder2_data,
        'PID right Speed Control': pid_speed_control_right_data,
        'PID left Speed Control': pid_speed_control_left_data,
        'Motor Speed 1': motor_speed1_data,
        'Motor Speed 2': motor_speed2_data,
    }

    csv_file_name = f'motor_data_forward_{target_pulses}pulses_kp{kp}_ki{ki}_kd{kd}.csv'  # Choose a suitable file name

    try:
        with open(csv_file_name, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=data.keys())
            writer.writeheader()
            for i in range(len(encoder1_data)):
                row = {key: data[key][i] for key in data.keys()}
                writer.writerow(row)

        print(f'Data saved to {csv_file_name}')
    except Exception as e:
        print(f'Error saving data to CSV: {e}')
