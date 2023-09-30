import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

dip_switch_1 = 13
dip_switch_2 = 6

rotary_switch_1 = 10
rotary_switch_2 = 9
rotary_switch_4 = 11
rotary_switch_8 = 8


GPIO.setup(dip_switch_1, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(dip_switch_2, GPIO.IN, pull_up_down = GPIO.PUD_UP)

GPIO.setup(rotary_switch_1, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(rotary_switch_2, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(rotary_switch_4, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(rotary_switch_8, GPIO.IN, pull_up_down = GPIO.PUD_UP)


   
    
while (1):
    time.sleep(0.5)
    if GPIO.input(dip_switch_1) == GPIO.HIGH:
        print("Dip Switch 1 = OFF")
    if GPIO.input(dip_switch_1) == GPIO.LOW:
        print("Dip Switch 1 = ON")
        
    if GPIO.input(dip_switch_2) == GPIO.HIGH:
        print("Dip Switch 2 = OFF")
    if GPIO.input(dip_switch_2) == GPIO.LOW:
        print("Dip Switch 2 = ON")
 
 
    if GPIO.input(rotary_switch_1) == GPIO.HIGH:
        print("Rotary Switch 1 = OFF")
    if GPIO.input(rotary_switch_1) == GPIO.LOW:
        print("Rotary Switch 1 = ON")
        
    if GPIO.input(rotary_switch_2) == GPIO.HIGH:
        print("Rotary Switch 2 = OFF")
    if GPIO.input(rotary_switch_2) == GPIO.LOW:
        print("Rotary Switch 2 = ON") 
        
    if GPIO.input(rotary_switch_4) == GPIO.HIGH:
        print("Rotary Switch 4 = OFF")
    if GPIO.input(rotary_switch_4) == GPIO.LOW:
        print("Rotary Switch 4 = ON")  
            
    if GPIO.input(rotary_switch_8) == GPIO.HIGH:
        print("Rotary Switch 8 = OFF")
    if GPIO.input(rotary_switch_8) == GPIO.LOW:
        print("Rotary Switch 8 = ON")
        
        
