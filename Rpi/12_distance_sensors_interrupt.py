import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

sensor_left = 16
sensor_right = 23


GPIO.setup(sensor_left, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(sensor_right, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def sensor_left(Channel):
    print("L-HIGH-CLOSED")
    
def sensor_right(Channel):
    print("R-HIGH-CLOSED")

GPIO.add_event_detect(16, GPIO.RISING, callback = sensor_left)
GPIO.add_event_detect(23, GPIO.RISING, callback = sensor_right)



while (1):
    print("---------")
    time.sleep(0.5)
    print("---**----")
    time.sleep(0.5)
