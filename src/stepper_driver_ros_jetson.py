#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import time
from math import pi
from math import copysign

PINS_CONFIG = {"STEP_L": 18, "DIR_L": 4,
               "STEP_R": 20, "DIR_R": 16}


class motors ():
    def __init__(self, pins_config, init_speed=0):
        self.leftSub = rospy.Subscriber(
            "/stepper_cmd", Int16MultiArray, self.stepper_callback)
        #self.pi = pigpio_istance
        self.pins_config = pins_config
        self.init_pins()
        self.l_speed = init_speed
        self.r_speed = init_speed
        self.left_dir = bool(0)
        self.right_dir = bool(0)
        self.last_time_check_step_l = time.perf_counter()*1e6
        self.last_time_check_step_r = time.perf_counter()*1e6
        self.step_cur_state_l = bool(0)
        self.step_cur_state_r = bool(0)
    def init_pins(self):
        for key in self.pins_config:
            GPIO.setup(self.pins_config[key], GPIO.OUT)
            GPIO.output(self.pins_config[key], GPIO.LOW)

    def stepper_callback(self, msg):
        print("new data received")
        received_d=msg.data
        right_d=bool(received_d[0])
        left_d=bool(received_d[1])
        self.r_speed=received_d[2]
        self.l_speed=received_d[3]
        if(self.right_dir != right_d):
            self.right_dir = right_d
            GPIO.output(self.pins_config["DIR_R"], right_d)
        if(self.left_dir != left_d):
            self.left_dir = left_d
            GPIO.output(self.pins_config["DIR_L"], left_d)

    def update_speed(self):
        
        #self.pi.get_current_tick()
        t_us_now = time.perf_counter()*1e6

        if (t_us_now- self.last_time_check_step_l >=self.l_speed) and (self.l_speed > 100):
            self.step_cur_state_l = not self.step_cur_state_l
            GPIO.output(self.pins_config["STEP_L"], self.step_cur_state_l)
            self.last_time_check_step_l = t_us_now
        if (t_us_now- self.last_time_check_step_r >=self.r_speed) and (self.r_speed > 100):
            self.step_cur_state_r= not self.step_cur_state_r
            GPIO.output(self.pins_config["STEP_R"], self.step_cur_state_r)
            self.last_time_check_step_r = t_us_now


if __name__ == '__main__':
    try:
        rospy.init_node('motors_control', anonymous=True)
        GPIO.setmode(GPIO.BCM)
        rate = rospy.Rate(100000)
        steppers = motors(PINS_CONFIG, init_speed=0)
#  
        while not rospy.is_shutdown():
            steppers.update_speed()
            rate.sleep()
        GPIO.cleanup()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
