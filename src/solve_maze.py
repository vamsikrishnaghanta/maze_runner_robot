import time

import main



# Function to follow the right wall
def follow_right_wall():
    while True:
        front_sensor, right_sensor, _ = check_sensors()

        if not right_sensor:  # No wall on the right
            motor_controls.rotate_in_degrees('right', motor_controls.pulse_for_90_degree)
            return

        if not front_sensor:  # No wall in front
            motor_controls.drive_straight()
            time.sleep(1)
            motor_controls.rotate_in_degrees('left', motor_controls.pulse_for_90_degree)

        motor_controls.drive_straight()


# Function to handle cases
def solve_maze():
    while True:
        front_sensor, right_sensor, left_sensor = check_sensors()

        if not front_sensor and right_sensor:
            # Case 1: Follow the right wall
            follow_right_wall()
        elif not right_sensor:
            # Case 2: Turn right and move forward
            motor_controls.rotate_in_degrees('right', motor_controls.pulse_for_90_degree)
            motor_controls.drive_straight()
        elif front_sensor and right_sensor and left_sensor:
            # Case 3: Dead-end, rotate 180 degrees
            motor_controls.rotate_in_degrees('right', motor_controls.pulse_for_180_degree)

# Start solving the maze
solve_maze()
