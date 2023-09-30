import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

duty_cycle = 0

LED = 7

Motor1_PWM = 18
Motor1_IN1 = 17
Motor1_IN2 = 22

Motor2_PWM = 19
Motor2_IN1 = 24
Motor2_IN2 = 4

def M1_forward():
    GPIO.output(Motor1_IN2,GPIO.LOW)
    GPIO.output(Motor1_IN1,GPIO.HIGH)
    
def M1_backward():
    GPIO.output(Motor1_IN1,GPIO.LOW)
    GPIO.output(Motor1_IN2,GPIO.HIGH)

def M2_forward():
    GPIO.output(Motor2_IN2,GPIO.LOW)
    GPIO.output(Motor2_IN1,GPIO.HIGH)
    
def M2_backward():
    GPIO.output(Motor2_IN1,GPIO.LOW)
    GPIO.output(Motor2_IN2,GPIO.HIGH)

# Using PWM in RPi.GPIO
# see https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/

GPIO.setup(Motor1_IN1,GPIO.OUT)
GPIO.setup(Motor1_IN2,GPIO.OUT)
GPIO.setup(Motor1_PWM,GPIO.OUT)
PWM_1 = GPIO.PWM(Motor1_PWM, 90) #GPIO als PWM mit Frequenz 90Hz
PWM_1.start(0) #Duty Cycle = 0

GPIO.setup(Motor2_IN1,GPIO.OUT)
GPIO.setup(Motor2_IN2,GPIO.OUT)
GPIO.setup(Motor2_PWM,GPIO.OUT)
PWM_2 = GPIO.PWM(Motor2_PWM, 90) #GPIO als PWM mit Frequenz 90Hz
PWM_2.start(0) #Duty Cycle = 0

GPIO.setup(LED, GPIO.OUT)
i = 0

while (1):
    print(i)
    i +=1
    GPIO.output(LED, GPIO.HIGH)
    
    PWM_1.ChangeDutyCycle(40)
    M1_forward()
    
    PWM_2.ChangeDutyCycle(40)
    M2_forward()
       
    time.sleep(5)
    
    GPIO.output(LED, GPIO.LOW)
    
    PWM_1.ChangeDutyCycle(80)
    
    PWM_2.ChangeDutyCycle(40)
    M2_backward()
    
    time.sleep(5)
    
    PWM_1.ChangeDutyCycle(0)
    PWM_2.ChangeDutyCycle(0)
        
    time.sleep(3)s
    