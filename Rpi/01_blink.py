import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LED = 7

GPIO.setup(LED, GPIO.OUT)
i = 0

while (1):
    print(i)
    i +=1
    GPIO.output(LED, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(LED, GPIO.LOW)
    time.sleep(1)
    
    
    
    