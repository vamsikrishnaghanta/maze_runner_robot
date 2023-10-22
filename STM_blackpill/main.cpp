#include <Arduino.h>

#define MOTOR_RIGHT_ENABLE_PIN 27
#define MOTOR_RIGHT_PIN1 5
#define MOTOR_RIGHT_PIN2 22

#define MOTOR_LEFT_ENABLE_PIN 6
#define MOTOR_LEFT_PIN1 4
#define MOTOR_LEFT_PIN2 17

#define ENCODER_RIGHT_C1 15
#define ENCODER_RIGHT_C2 14
#define ENCODER_LEFT_C1 21
#define ENCODER_LEFT_C2 20

#define SENSOR_FRONT 26
#define SENSOR_RIGHT 19
#define SENSOR_LEFT 13

void setup() {
  // Set up GPIO mode and pins
  pinMode(MOTOR_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN2, OUTPUT);

  pinMode(MOTOR_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);

  pinMode(ENCODER_RIGHT_C1, INPUT);
  pinMode(ENCODER_RIGHT_C2, INPUT);
  pinMode(ENCODER_LEFT_C1, INPUT);
  pinMode(ENCODER_LEFT_C2, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);

  // Initialize PWM for motor speed control
  analogWriteFrequency(MOTOR_RIGHT_ENABLE_PIN, 100); // Frequency = 100 Hz
  analogWriteFrequency(MOTOR_LEFT_ENABLE_PIN, 100);

  // Start PWM
  analogWrite(MOTOR_RIGHT_ENABLE_PIN, 0);  // Start with 0% duty cycle
  analogWrite(MOTOR_LEFT_ENABLE_PIN, 0);
}

void loop() {
  // Check sensors
  bool front_sensor = digitalRead(SENSOR_FRONT);
  bool right_sensor = digitalRead(SENSOR_RIGHT);
  bool left_sensor = digitalRead(SENSOR_LEFT);

  // If there is no wall in front, drive straight
  if (!front_sensor) {
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, 255);
    analogWrite(MOTOR_LEFT_ENABLE_PIN, 255);
  }

  // If there is a wall in front and to the right, rotate left
  else if (front_sensor && right_sensor) {
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, 0);
    analogWrite(MOTOR_LEFT_ENABLE_PIN, 255);
    delay(1000);
  }

  // If there is a wall in front and to the left, rotate right
  else if (front_sensor && left_sensor) {
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, 255);
    analogWrite(MOTOR_LEFT_ENABLE_PIN, 0);
    delay(1000);
  }

  // If there are walls on all sides, stop
  else {
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, 0);
    analogWrite(MOTOR_LEFT_ENABLE_PIN, 0);
  }
}
