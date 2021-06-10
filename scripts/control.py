#!/usr/bin/env python
import rospy
from jetbot import Heartbeat
from jetbot import Robot
from maze_buster.msg import Motor
from geometry_msgs.msg import Twist
import numpy as np

STEERING_AND_THROTTLE_NODE_NAME = 'cmd_vel_node'
MOVEMENT_TOPIC_NAME = 'cmd_vel'

robot = Robot()
robot.left_motor.alpha = 1
robot.right_motor.alpha = 0.84

def callback(data):
    move = np.array([data.linear.x, data.linear.x])
    rotate = np.array([data.angular.z, -data.angular.z]) * 0.45
    if data.angular.z == 0:
        robot.left_motor.value, robot.right_motor.value = move
    else:
        robot.left_motor.value, robot.right_motor.value = rotate
        

def listener():
    rospy.init_node(STEERING_AND_THROTTLE_NODE_NAME, anonymous=False)
    rospy.Subscriber(MOVEMENT_TOPIC_NAME, Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
