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
target_pulses = 50  # Adjust as needed
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



turn_direction ='left'

try:
    while encoder_right_count_c1 < target_pulses and encoder_right_count_c2 < target_pulses:
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

        # Set motor directions based on turn direction
        if turn_direction == 'left':
            GPIO.output(motor1_pin1, GPIO.HIGH)
            GPIO.output(motor1_pin2, GPIO.LOW)
            GPIO.output(motor2_pin1, GPIO.LOW)
            GPIO.output(motor2_pin2, GPIO.HIGH)
        elif turn_direction == 'right':
            GPIO.output(motor1_pin1, GPIO.LOW)
            GPIO.output(motor1_pin2, GPIO.HIGH)
            GPIO.output(motor2_pin1, GPIO.HIGH)
            GPIO.output(motor2_pin2, GPIO.LOW)
        else:
            print("Invalid turn direction")

        control_signal1_data.append(control_signal1)
        control_signal2_data.append(control_signal2)
        pid_speed_control_data.append(pid_speed_control_signal)
        motor_speed1_data.append(motor_speed1)
        motor_speed2_data.append(motor_speed2)
        encoder1.append(encoder_right_count_c1)
        encoder2.append(encoder_left_count_c1)

        # Set the motor speeds
        set_motor_speeds(motor_speed1, motor_speed2)

        previous_error1 = error1
        previous_error2 = error2
        previous_speed_error = speed_error

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

    csv_file_name = f'motor_data_turn_{target_pulses}pulses_kp{kp}_ki{ki}_kd{kd}.csv'  # Choose a suitable file name

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


