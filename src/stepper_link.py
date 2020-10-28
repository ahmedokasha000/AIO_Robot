#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import math as m

STEPS_PER_REV = 400
W_DIAMETER = 0.065
WHEEL_SEP = 0.2
ROBOT_DIAMETER = 0.36


class SteppersLink():
    RIGHT_DIR = 0
    LEFT_DIR = 1
    RIGHT_DEL = 2
    LEFT_DEL = 3
    result_array = [0, 0, 0, 0]  # right_dir ,left_dir ,right_delay, left_delay

    def __init__(self, steps_per_rev, wheel_diameter, wheel_seperation):
        self.steps_per_rev = steps_per_rev
        self.wheel_diameter = wheel_diameter
        self.wheel_seperation = wheel_seperation
        self.cmdSub = rospy.Subscriber("/cmd_vel", Twist, self.process_cmd)
        self.stepperPub = rospy.Publisher(
            "stepper_cmd", Int32MultiArray, queue_size=1)

    @staticmethod
    def get_del_time_us(diameter, speed, steps_per_rev):
        wait_bet_high_low_us = (1e6*m.pi*diameter)/(speed*steps_per_rev*2)
        return int(wait_bet_high_low_us)

    def process_cmd(self, msg):
        if (msg.linear.x != 0.0 or msg.angular.z != 0.0):
            speed_wish_right = msg.linear.x + \
                (msg.angular.z * ROBOT_DIAMETER)/2.
            speed_wish_left = msg.linear.x - \
                (msg.angular.z * ROBOT_DIAMETER)/2.
            if speed_wish_right < 0:
                self.result_array[self.RIGHT_DIR] = 1
            else:
                self.result_array[self.RIGHT_DIR] = 0
            if speed_wish_left < 0:
                self.result_array[self.LEFT_DIR] = 1
            else:
                self.result_array[self.LEFT_DIR] = 0
            speed_wish_right = abs(speed_wish_right)
            speed_wish_left = abs(speed_wish_left)
            self.result_array[self.RIGHT_DEL] = self.get_del_time_us(
                self.wheel_diameter, speed_wish_right, self.steps_per_rev)
            self.result_array[self.LEFT_DEL] = self.get_del_time_us(
                self.wheel_diameter, speed_wish_left, self.steps_per_rev)
        else:
            self.result_array = [0, 0, 0, 0]
        mes_to_motors = Int32MultiArray()
        mes_to_motors.data = self.result_array
        self.stepperPub.publish(mes_to_motors)


if __name__ == '__main__':
    try:
        rospy.init_node('stepper_link', anonymous=True)
        motors_link = SteppersLink(STEPS_PER_REV, W_DIAMETER, WHEEL_SEP)
        # rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
