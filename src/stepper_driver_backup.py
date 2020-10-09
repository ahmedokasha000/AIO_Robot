#!/usr/bin/env python3
import RPi.GPIO as GPIO
import pigpio
import rospy
from std_msgs.msg import String
from math import pi
from math import copysign

PINS_CONFIG = {"STEP_L": 18, "DIR_L": 4,
               "STEP_R": 20, "DIR_R": 16}


class motors ():
    def __init__(self, pins_config, pigpio_istance, init_speed=0):
        self.leftSub = rospy.Subscriber(
            "/stepper_cmd", String, self.stepper_callback)
        self.pi = pigpio_istance
        self.pins_config = pins_config
        self.init_pins()
        self.speed = init_speed
        self.left_dir = bool(0)
        self.right_dir = bool(0)
        self.last_time_check_step = pi.get_current_tick()
        self.step_cur_state = bool(0)

    def init_pins(self):
        for key in self.pins_config:
            GPIO.setup(self.pins_config[key], GPIO.OUT)
            GPIO.output(self.pins_config[key], GPIO.LOW)

    def stepper_callback(self, msg):
        print("new data received")
        received_data = (msg.data).split(',')
        self.speed = int(received_data[0])
        right_d = bool(int(received_data[1]))
        left_d = bool(int(received_data[2]))
        if(self.right_dir != right_d):
            self.right_dir = right_d
            GPIO.output(self.pins_config["DIR_R"], right_d)
        if(self.left_dir != left_d):
            self.left_dir = left_d
            GPIO.output(self.pins_config["DIR_L"], left_d)

    def update_speed(self):
        t_us_now = self.pi.get_current_tick()
        if (t_us_now >= self.last_time_check_step+self.speed) and (self.speed != 0):
            self.step_cur_state = not self.step_cur_state
            GPIO.output(self.pins_config["STEP_L"], self.step_cur_state)
            GPIO.output(self.pins_config["STEP_R"], self.step_cur_state)
            self.last_time_check_step = t_us_now


if __name__ == '__main__':
    try:
        rospy.init_node('motors_control', anonymous=True)
        pi = pigpio.pi()
        GPIO.setmode(GPIO.BCM)
        rate = rospy.Rate(100000)
        steppers = motors(PINS_CONFIG, pi, init_speed=2000)

        while not rospy.is_shutdown():
            steppers.update_speed()
            rate.sleep()
        GPIO.cleanup()
    except rospy.ROSInterruptException:
        GPIO.cleanup()