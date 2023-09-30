import RPi.GPIO as GPIO
import time

# Define the motor control pins
motor1_enable_pin = 27  # PWM pin for Motor 1 speed control
motor1_pin1 = 5  # Motor 1 input pin 1
motor1_pin2 = 22  # Motor 1 input pin 2

motor2_enable_pin = 6  # PWM pin for Motor 2 speed control
motor2_pin1 = 4  # Motor 2 input pin 1
motor2_pin2 = 17  # Motor 2 input pin 2

# Define encoder pins (c1 and c2) for both motors
encoder1_c1 = 15
encoder1_c2 = 14
encoder2_c1 = 21
encoder2_c2 = 20
current_time = time.time()

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


# Variables for encoder readings
encoder_right_count_c1 = 0
encoder_right_count_c2 = 0
encoder_left_count_c1 = 0
encoder_left_count_c2 = 0

target_pulses = 100


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

        print(f'pulse_counter1:{encoder_right_count_c1}, pulse_counter2: {encoder_left_count_c1}')

        # Adjust the motor speeds based on the synchronization control signal
        motor_speed1 = 20
        motor_speed2 = 20

        print(f'motor_speed1:{motor_speed1}, motor_speed2: {motor_speed2}')

        # Set motor directions for forward movement
        GPIO.output(motor1_pin1, GPIO.HIGH)
        GPIO.output(motor1_pin2, GPIO.LOW)
        GPIO.output(motor2_pin1, GPIO.HIGH)
        GPIO.output(motor2_pin2, GPIO.LOW)

        # Set the motor speeds
        set_motor_speeds(motor_speed1, motor_speed2)

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
