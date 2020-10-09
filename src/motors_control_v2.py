#!/usr/bin/env python3
import RPi.GPIO as GPIO
from time import sleep, time
import rospy
from geometry_msgs.msg import Twist
from math import pi
from math import copysign


STEP1 = 18
DIR1 = 4
STEP2 = 22
DIR2 = 27
STEP3 = 20
DIR3 = 16  # 24
STEP4 = 6
DIR4 = 5
WHEEL_SEP = 0.22
WHEEL_D = 0.06
left_delay = 0.0
right_delay = 0.0
last_dir_right = 1.0
last_dir_left = 1.0
last_check_right = 0.0
last_check_left = 0.0


def motors_init():
    global STEP1, STEP2, STEP3, STEP4, DIR1, DIR2, DIR3, DIR4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(STEP1, GPIO.OUT)
    GPIO.setup(DIR1, GPIO.OUT)
    GPIO.setup(STEP2, GPIO.OUT)
    GPIO.setup(DIR2, GPIO.OUT)
    GPIO.setup(STEP3, GPIO.OUT)
    GPIO.setup(DIR3, GPIO.OUT)
    GPIO.setup(STEP4, GPIO.OUT)
    GPIO.setup(DIR4, GPIO.OUT)
    GPIO.output(DIR1, GPIO.LOW)
    GPIO.output(DIR2, GPIO.LOW)
    GPIO.output(DIR3, GPIO.LOW)
    GPIO.output(DIR4, GPIO.LOW)


def motors_move():
    global STEP1, STEP2, STEP3, STEP4, DIR1, DIR2, DIR3, DIR4
    global left_delay, right_delay, last_check_right, last_check_left
    time_now = time()

    if (time_now > last_check_right+abs(right_delay)):
        GPIO.output(STEP1, not GPIO.input(STEP1))  # toggle
        GPIO.output(STEP2, not GPIO.input(STEP2))  # toggle
        last_check_right = time_now
    if (time_now > last_check_left+abs(left_delay)):
        GPIO.output(STEP3, not GPIO.input(STEP3))  # toggle
        GPIO.output(STEP4, not GPIO.input(STEP4))  # toggle
        last_check_left = time_now


def motors_mov_dir():
    global STEP1, STEP2, STEP3, STEP4, DIR1, DIR2, DIR3, DIR4
    global left_delay, right_delay, last_dir_left, last_dir_right
    if last_dir_left == copysign(1.0, left_delay):
        pass
    elif last_dir_left == 1.0 and copysign(1.0, left_delay) == -1.0:
        GPIO.output(DIR3, GPIO.HIGH)
        GPIO.output(DIR4, GPIO.HIGH)
        last_dir_left = -1.0
        print("left part backward")
    elif last_dir_left == -1.0 and copysign(1.0, left_delay) == 1.0:
        GPIO.output(DIR3, GPIO.LOW)
        GPIO.output(DIR4, GPIO.LOW)
        last_dir_left = 1.0
        print("left part forward")

    if last_dir_right == copysign(1.0, right_delay):
        pass
    elif last_dir_right == 1.0 and copysign(1.0, right_delay) == -1.0:
        GPIO.output(DIR1, GPIO.HIGH)
        GPIO.output(DIR2, GPIO.HIGH)
        last_dir_right = -1.0
        print("right part backward")

    elif last_dir_right == -1.0 and copysign(1.0, right_delay) == 1.0:
        GPIO.output(DIR1, GPIO.LOW)
        GPIO.output(DIR2, GPIO.LOW)
        last_dir_right = 1.0
        print("right part forward")
    if (right_delay != 0.0 and left_delay != 0.0):
        motors_move()


def motorsCallback(msg):
    global WHEEL_SEP, WHEEL_D, right_delay, left_delay
    if (msg.linear.x != 0.0 or msg.angular.z != 0.0):
        pass
        speed_wish_right = msg.linear.x+(msg.angular.z * WHEEL_SEP)/2.
        speed_wish_left = msg.linear.x-(msg.angular.z * WHEEL_SEP)/2.
        right_delay = (pi * WHEEL_D) / (400 * speed_wish_right)
        left_delay = (pi * WHEEL_D) / (400 * speed_wish_left)
    else:
        pass
        right_delay = 0.0
        left_delay = 0.0


if __name__ == '__main__':
    try:
        motors_init()
        rospy.init_node('motors_control', anonymous=True)
        cmdSub = rospy.Subscriber("/cmd_vel", Twist, motorsCallback)
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            motors_mov_dir()
            rate.sleep()
        GPIO.cleanup()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
