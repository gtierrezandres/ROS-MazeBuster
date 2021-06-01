#!/usr/bin/env python
import rospy
from jetbot import Robot
from jetbot import Heartbeat
from sensor_msgs.msg import LaserScan
from maze_buster.msg import Motor
from std_msgs.msg import Float32
import numpy as np

STEERING_AND_THROTTLE_TOPIC_NAME = 'cmd_vel'
LIDAR_TOPIC_NAME = 'scan'
CALCULATIONS_NODE_NAME = 'wall_node'
THROTTLE_CONSTANT = 0.4
STEERING_CONSTANT = 0.4
DIST_THESHOLD = 0.3
error = 0.0
sum_error = 0.0
Kp= 1.0
Ki= 0.5
Kd= 0.1

def move_forward():
    left_motor.data = THROTTLE_CONSTANT
    right_motor.data = THROTTLE_CONSTANT
    motors = [left_motor.data, right_motor.data]
    # print("this is motors: ", motors)
    return motors

def turn_left_in_place(laser_reading):
    left_motor.data =  STEERING_CONSTANT
    right_motor.data = STEERING_CONSTANT
    motors = [left_motor.data, right_motor.data]
    return motors

def turn_right_in_place(laser_reading):
    left_motor.data = STEERING_CONSTANT
    right_motor.data = 0.5*STEERING_CONSTANT
    motors = [left_motor.data, right_motor.data]
    return motors

def wall_following(laser_reading):
        left_motor.data = STEERING_CONSTANT
        right_motor.data = (laser_reading)*STEERING_CONSTANT
        motors = [left_motor.data, right_motor.data]
        return motors

def robotCalculations(data):
    target_range = data.ranges[300:420]
    mid_point = data.ranges[int(len(target_range)/2)]
    min_dist = min(target_range)
    error = DIST_THESHOLD-min_dist
    laser_reading =1+Kp*error+Ki*prev_error+Kd*sum_error
    movement_pub.publish(wall_following(laser_reading))

if __name__ == '__main__':
    left_motor = Float32()
    left_motor.data = 0.0

    right_motor = Float32()
    right_motor.data = 0.0

    prev_error = error
    sum_error = sum_error + error

    motors = Motor()

    rospy.init_node(CALCULATIONS_NODE_NAME, anonymous=False)
    rospy.Subscriber(LIDAR_TOPIC_NAME, LaserScan, robotCalculations)
    movement_pub = rospy.Publisher(STEERING_AND_THROTTLE_TOPIC_NAME, Motor, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    
        rospy.spin()
        rate.sleep()
