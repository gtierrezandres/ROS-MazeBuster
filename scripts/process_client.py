#!/usr/bin/env python
import rospy
from jetbot import Robot
from jetbot import Heartbeat
from sensor_msgs.msg import LaserScan
from maze_buster.msg import Motor
from std_msgs.msg import Float32
import numpy as np
import math 

STEERING_AND_THROTTLE_TOPIC_NAME = 'cmd_vel'
LIDAR_TOPIC_NAME = 'scan'
PROCESS_NODE_NAME = 'process_node'
THROTTLE_CONSTANT = 0.45
STEERING_CONSTANT = 0.45
DIST_THESHOLD = 0.2
fw_error = 0.0
lw_error = 0.0
fw_sum_error = 0.0
lw_sum_error = 0.0
Kp= 3
Ki= 0.1
Kd= 0.7


# def move_forward():
#     left_motor.data = THROTTLE_CONSTANT
#     right_motor.data = THROTTLE_CONSTANT
#     motors = [left_motor.data, right_motor.data]
#     # print("this is motors: ", motors)
#     return motors


# def turn_left_in_place(laser_reading):
#     left_motor.data = -STEERING_CONSTANT
#     right_motor.data = STEERING_CONSTANT
#     motors = [left_motor.data, right_motor.data]
#     return motors
#     # while laser_reading <= DIST_THESHOLD:
#     #     robot.set_motors(-STEERING_CONSTANT, STEERING_CONSTANT)
    
# def turn_right_in_place(laser_reading):
#     left_motor.data = STEERING_CONSTANT
#     right_motor.data = -STEERING_CONSTANT
#     motors = [left_motor.data, right_motor.data]
#     return motors
#     # while laser_reading <= DIST_THESHOLD:
#     #     robot.set_motors(STEERING_CONSTANT, -STEERING_CONSTANT)

def processDistances(fw, lw):
    left_motor.data = (lw) * STEERING_CONSTANT
    right_motor.data = STEERING_CONSTANT
    motors = [left_motor.data, right_motor.data]
    movement_pub.publish(motors)


def getDistances(data):
    rate.sleep()
    fw_distance = data.ranges[480:600]
    lw_distance = data.ranges[705:] + data.ranges[:15]
    
    lw_distance = [x if not math.isinf(x) else 0.1 for x in lw_distance]
    # print(lw_distance)
    left = lw_distance[0]
    right = lw_distance[len(lw_distance)-1]
    alpha = 0
    dist = 0

    if abs(min(lw_distance)-left) <= 0.002:
        alpha = np.arctan((np.cos(15)*right-left)/np.sin(15))
        dist = left*np.cos(alpha)

    elif abs(min(lw_distance)-right) <= 0.002:
        alpha = np.arctan((np.cos(15)*left-right)/np.sin(15))
        dist = right*np.cos(alpha)

    else:
        dist = min(lw_distance)

    fw_error = DIST_THESHOLD - dist
    fw_laser_reading = 1 + Kp * fw_error + Ki * fw_prev_error + Kd * fw_sum_error

    lw_error = DIST_THESHOLD - dist
    lw_laser_reading = 1 + Kp * lw_error + Ki * lw_prev_error + Kd * lw_sum_error
    processDistances(fw_laser_reading, lw_laser_reading)

    # target_range = data.ranges[480:600]
    # mid_point = data.ranges[int(len(target_range)/2)]
    # for dist in range(len(target_range)):
    #     if target_range[dist] < DIST_THESHOLD:
    #         laser_reading = target_range[dist]
    #         if dist <= mid_point:
    #             movement_pub.publish(turn_left_in_place(laser_reading))
    #         elif dist >= mid_point:
    #             movement_pub.publish(turn_right_in_place(laser_reading))

    #     else:
    #         movement_pub.publish(move_forward())
                


if __name__ == '__main__':
    left_motor = Float32()
    left_motor.data = 0.0

    right_motor = Float32()
    right_motor.data = 0.0

    fw_prev_error = fw_error
    fw_sum_error = fw_sum_error + fw_error

    lw_prev_error = lw_error
    lw_sum_error = lw_sum_error + lw_error

    motors = Motor()

    rospy.init_node(PROCESS_NODE_NAME, anonymous=False)
    rospy.Subscriber(LIDAR_TOPIC_NAME, LaserScan, getDistances)
    movement_pub = rospy.Publisher(STEERING_AND_THROTTLE_TOPIC_NAME, Motor, queue_size=1)
    rate = rospy.Rate(150)
    rospy.spin()