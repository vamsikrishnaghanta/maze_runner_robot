import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LED = 7

GPIO.setup(LED, GPIO.OUT)

while (1):
        key = input("Eingabe")
        print (key)
        if key == "e":
            GPIO.output(LED, GPIO.HIGH)
        if key == "a":
            GPIO.output(LED, GPIO.LOW)
        if key == "q":
            exit()
