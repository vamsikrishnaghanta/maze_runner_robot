import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

sensor_left = 16
sensor_right = 23


GPIO.setup(sensor_left, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(sensor_right, GPIO.IN, pull_up_down = GPIO.PUD_UP)

# Pressing the pushbutton (Taster) LED is off
while (1):
        if GPIO.input(sensor_left) == GPIO.HIGH:
            print("L-HIGH-CLOSED")
        if GPIO.input(sensor_left) == GPIO.LOW:
            print("L-LOW-OPEN")
            
        if GPIO.input(sensor_right) == GPIO.HIGH:
            print("R-HIGH-CLOSED")
        if GPIO.input(sensor_right) == GPIO.LOW:
            print("R-LOW-OPEN")
            
        time.sleep(1)
