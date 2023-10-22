# Maze Runner Robot

![4 isometric](https://github.com/vamsikrishnaghanta/maze_runner_robot/assets/145189721/a48ee95b-d255-4e9a-96c1-8ab498fdd558)


## Description

This repository contains the code for a maze runner robot built using the Raspberry Pi (RPi) and an STM32 Black Pill microcontroller. The robot is designed to autonomously navigate through a maze by utilizing sensors, motor controls, and PID controllers. It can move forward, rotate, and detect obstacles in its path.

The robot's primary components and functionalities include:

- **Motor Control**: The robot features two motors for driving the wheels, with control over speed and direction using PWM signals.

- **Encoder Feedback**: Encoders are used to provide feedback on the motor's rotation, enabling precise control and navigation.

- **IR Sensors**: Infrared (IR) sensors are employed to detect obstacles in front, right, and left directions.

- **PID Controllers**: The robot utilizes PID (Proportional-Integral-Derivative) controllers to maintain straight-line motion and accurately execute rotations.

- **Autonomous Navigation**: It can autonomously drive forward and rotate by specific degrees to navigate through a maze.

## Code Structure

The code is structured as follows:

- `main.py`: The main script that controls the robot's behavior, including PID control, sensor monitoring, and maze-solving algorithms.

- `MotorControls`: A class that handles motor control, PID control, and encoder feedback for the robot.

- `check_sensors`: A function that reads data from IR sensors to detect obstacles.

- `solve_maze`: A function that implements maze-solving logic based on sensor input and navigation algorithms.

## Usage

You can use this code to build and program your own maze runner robot. The main script, `main.py`, provides an example of how to use the `MotorControls` class and implement maze-solving logic in the `solve_maze` function.

## Dependencies

The code relies on the following dependencies:

- [RPi.GPIO](https://pypi.org/project/RPi.GPIO/): A Python library for Raspberry Pi GPIO control.

Ensure that you have the necessary dependencies installed to run the code successfully.

## Getting Started

1. Assemble the hardware components of the maze runner robot.

2. Install the required dependencies, including RPi.GPIO.

3. Configure the GPIO pins for motor control, encoders, and sensors as per your robot's hardware setup.

4. Run the `main.py` script to start the robot's maze-solving operation.

## License

Feel free to customize and extend the code to suit your specific maze runner robot project. Good luck with your maze-solving adventures!

