#!/usr/bin/env python3
import RPi.GPIO as GPIO
import pigpio
from time import sleep, time
pi = pigpio.pi()
STEP1 = 18
DIR1 = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.setup(DIR1, GPIO.OUT)
GPIO.output(DIR1, GPIO.HIGH)
GPIO.output(STEP1, GPIO.HIGH)
speed = 100
time_st = pi.get_current_tick()
count = 0
try:
    while (1):
        if (pi.get_current_tick() > time_st+speed):
            GPIO.output(STEP1, not GPIO.input(STEP1))
            count += 1
            time_st = pi.get_current_tick()
        if(count > 800):
            print("speed = ", speed)
            speed += 100
            count = 0
except :
    GPIO.cleanup()
