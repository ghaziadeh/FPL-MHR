import RPi.GPIO as GPIO
import time
from time import sleep
import sys
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
from math import floor
GPIO.setup(25, GPIO.OUT)
class Motor1():
    def __init__(self,PWM,Dir):
        self.PWM = PWM
        self.Dir = Dir
        GPIO.setup(self.PWM,GPIO.OUT)
        GPIO.setup(self.Dir,GPIO.OUT)
        self.pwm = GPIO.PWM(self.PWM,100)
        self.pwm.start(0)

    def moveF(self,x=35):
        GPIO.output(self.Dir,GPIO.HIGH)
        self.pwm.ChangeDutyCycle(x)
        GPIO.output(25, GPIO.LOW)
        # sleep(0.001)


    def moveB(self,x=35):
        GPIO.output(self.Dir,GPIO.LOW)
        self.pwm.ChangeDutyCycle(x)
        GPIO.output(25, GPIO.LOW)
        #sleep(0.001)


    def stop(self,x=0):
        self.pwm.ChangeDutyCycle(x)


# Enable pins for IN1-4 to control step sequence
servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
def retract_hook():
    servoPIN = 4
    GPIO.setup(servoPIN, GPIO.OUT)
    IN2 = 25
    GPIO.output(IN2, GPIO.HIGH)
    p = GPIO.PWM(servoPIN, 50) # GPIO 2 for PWM with 50Hz
    p.start(7)#starting position, bottom poiting down
    print("start")
    time.sleep(1)
    p.ChangeDutyCycle(12)#pointing 180 degrees
    time.sleep(1)
    GPIO.output(IN2, GPIO.LOW)
#GPIO.output(4, 0)
coil_A_1_pin = 6
coil_A_2_pin = 13
coil_B_1_pin = 19
coil_B_2_pin = 26

GPIO.setup(coil_A_1_pin, GPIO.OUT)
GPIO.setup(coil_A_2_pin, GPIO.OUT)
GPIO.setup(coil_B_1_pin, GPIO.OUT)
GPIO.setup(coil_B_2_pin, GPIO.OUT)

# Set pin states
def setStep(w1, w2, w3, w4):
    GPIO.output(coil_A_1_pin, w1)
    GPIO.output(coil_A_2_pin, w2)
    GPIO.output(coil_B_1_pin, w3)
    GPIO.output(coil_B_2_pin, w4)


# loop through step sequence based on number of steps
def arm_retract(number_of_rotations = 6.4):
    try:
        servoPIN = 4
        GPIO.setup(servoPIN, GPIO.OUT)
        IN2 = 25
        GPIO.output(IN2, GPIO.HIGH)
        p = GPIO.PWM(servoPIN, 50) # GPIO 2 for PWM with 50Hz
        delay = 0.0055/7
        steps = floor(1380*number_of_rotations)
        setStep(1,1,1,1)
        p.start(12)
        print("start")
        time.sleep(1)
        for i in range(0, steps):
            p.ChangeDutyCycle(7)
            setStep(1,0,1,0)
            p.ChangeDutyCycle(7)
            time.sleep(delay)
            setStep(0,1,1,0)
            p.ChangeDutyCycle(7)
            time.sleep(delay)
            setStep(0,1,0,1)
            p.ChangeDutyCycle(7)
            time.sleep(delay)
            setStep(1,0,0,1)
            p.ChangeDutyCycle(7)
            time.sleep(delay)
        setStep(1,1,1,1)
    except KeyboardInterrupt:
        setStep(1,1,1,1)
        sys.exit()
        #delay -= 0.01
        # Reverse previous step sequence to reverse motor direction
def arm_extend(number_of_rotations = 6.4):
    try: 
        setStep(1,1,1,1)
        delay = 0.0055/7
        steps = floor(1380*number_of_rotations)
        for i in range(0, steps):
            setStep(1,0,0,1)
            time.sleep(delay)
            setStep(0,1,0,1)
            time.sleep(delay)
            setStep(0,1,1,0)
            time.sleep(delay)
            setStep(1,0,1,0)
            time.sleep(delay)
        setStep(1,1,1,1)
    except KeyboardInterrupt:
        setStep(1,1,1,1)
        sys.exit()
        
    


def extend_hook():
    servoPIN = 4
    GPIO.setup(servoPIN, GPIO.OUT)
    IN2 = 25
    GPIO.output(IN2, GPIO.HIGH)
    p = GPIO.PWM(servoPIN, 50) # GPIO 2 for PWM with 50Hz
    p.start(12)
    print("start")
    time.sleep(1)
    p.ChangeDutyCycle(7)
    time.sleep(1)
    
