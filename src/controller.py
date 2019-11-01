#!/usr/bin/env python
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
import Adafruit_PCA9685

pwm = None
STEERING_CHANNEL = 0
ACCELERATION_CHANNEL = 1
MAX_ACC = 1

def callback(data):
    global pwm

    steering = 0
    if data.steering_angle > 0:
        steering = min((data.steering_angle / (math.pi/2))*2048 + 2048, 4096)
    else:
        steering = max(2048 - (-data.steering_angle / (math.pi/2))*2048, 0)

    acceleration = data.acceleration / MAX_ACC * 4095

    rospy.logerr("Steering data: %lf", steering)
    rospy.logerr("Acceleration data: %lf", acceleration)

    pwm.set_pwm(STEERING_CHANNEL, 0, steering)
    pwm.set_pwm(ACCELERATION_CHANNEL, 0, acceleration)


def listener():
    global pwm

    rospy.init_node('mobilelab_controller')

    pwm = pwm = Adafruit_PCA9685.PCA9685()

    rospy.Subscriber('/mobilelab/cmd_vel', AckermannDrive, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()


