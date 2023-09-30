import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LED = 21

Motor1_PWM = 18
Motor1_IN1 = 17
Motor1_IN2 = 22

Motor2_PWM = 19
Motor2_IN1 = 24
Motor2_IN2 = 4

GPIO.setup(Motor1_PWM,GPIO.OUT)
GPIO.setup(Motor1_IN1,GPIO.OUT)
GPIO.setup(Motor1_IN2,GPIO.OUT)

GPIO.setup(Motor2_PWM,GPIO.OUT)
GPIO.setup(Motor2_IN1,GPIO.OUT)
GPIO.setup(Motor2_IN2,GPIO.OUT)


GPIO.setup(LED, GPIO.OUT)
i = 0

# both motors forward and backward
while (1):
    print(i)
    i +=1
    GPIO.output(LED, GPIO.HIGH)
    
    GPIO.output(Motor1_PWM,GPIO.HIGH)
    GPIO.output(Motor1_IN2,GPIO.LOW)
    GPIO.output(Motor1_IN1,GPIO.HIGH)
    
    GPIO.output(Motor2_PWM,GPIO.HIGH)
    GPIO.output(Motor2_IN2,GPIO.LOW)
    GPIO.output(Motor2_IN1,GPIO.HIGH)
    
    time.sleep(1)
    
    GPIO.output(Motor1_PWM,GPIO.LOW)
    GPIO.output(Motor1_IN2,GPIO.HIGH)
    GPIO.output(Motor1_IN1,GPIO.HIGH)
    
    GPIO.output(Motor2_PWM,GPIO.LOW)
    GPIO.output(Motor2_IN2,GPIO.HIGH)
    GPIO.output(Motor2_IN1,GPIO.HIGH)
    
    time.sleep(0.5)
    
    GPIO.output(LED, GPIO.LOW)
    
    GPIO.output(Motor1_PWM,GPIO.HIGH)
    GPIO.output(Motor1_IN1,GPIO.LOW)
    GPIO.output(Motor1_IN2,GPIO.HIGH)
    
    GPIO.output(Motor2_PWM,GPIO.HIGH)
    GPIO.output(Motor2_IN1,GPIO.LOW)
    GPIO.output(Motor2_IN2,GPIO.HIGH)
    
    time.sleep(1)
    
    GPIO.output(Motor1_PWM,GPIO.LOW)
    GPIO.output(Motor1_IN2,GPIO.HIGH)
    GPIO.output(Motor1_IN1,GPIO.HIGH)
    
    GPIO.output(Motor2_PWM,GPIO.LOW)
    GPIO.output(Motor2_IN2,GPIO.HIGH)
    GPIO.output(Motor2_IN1,GPIO.HIGH)
    
    time.sleep(0.5)
    