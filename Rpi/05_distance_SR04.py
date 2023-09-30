import RPi.GPIO as GPIO

from gpiozero import DistanceSensor
from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LED = 7
i = 0

GPIO.setup(LED, GPIO.OUT)

sensor = DistanceSensor(echo=27, trigger=25)

# Display the distance and LED is blinking
while True:
    print("Distance: ", sensor.distance * 100)
   
    GPIO.output(LED, GPIO.HIGH)
    sleep(1)
    GPIO.output(LED, GPIO.LOW)
    sleep(1)
    