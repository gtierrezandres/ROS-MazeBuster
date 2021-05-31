#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from jetbot import Heartbeat
from jetbot import Robot
from maze_buster.msg import Motor

STEERING_AND_THROTTLE_NODE_NAME = 'cmd_vel_node'
MOVEMENT_TOPIC_NAME = 'cmd_vel'

def callback(data):
    robot.left_motor, robot.right_motor = data.data

def listener():
    robot = Robot()
    robot.left_motor.alpha = 1
    robot.right_motor.alpha = 0.84
    rospy.init_node(STEERING_AND_THROTTLE_NODE_NAME, anonymous=False)
    rospy.Subscriber(MOVEMENT_TOPIC_NAME, Motor, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
