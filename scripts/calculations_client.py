#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from jetbot import Robot
from jetbot import Heartbeat
from sensor_msgs.msg import LaserScan
from maze_buster.msg import Motor

STEERING_AND_THROTTLE_TOPIC_NAME = 'cmd_vel'
LIDAR_TOPIC_NAME = 'scan'
CALCULATIONS_NODE_NAME = 'calculations_node'
THROTTLE_CONSTANT = 0.3
STEERING_CONSTANT = 0.3
DIST_THESHOLD = 0.1

left_motor = Float32()
left_motor.data = 0.0

right_motor = Float32()
right_motor.data = 0.0

motors = Motor()

def move_forward():
    left_motor.data = THROTTLE_CONSTANT
    right_motor.data = THROTTLE_CONSTANT
    motors = {left_motor, right_motor}
    return motors


def turn_left_in_place(laser_reading):
    left_motor.data = -STEERING_CONSTANT
    right_motor.data = STEERING_CONSTANT
    motors = {left_motor, right_motor}
    # while laser_reading <= DIST_THESHOLD:
    #     robot.set_motors(-STEERING_CONSTANT, STEERING_CONSTANT)
    
def turn_left_in_place(laser_reading):
    left_motor.data = STEERING_CONSTANT
    right_motor.data = -STEERING_CONSTANT
    motors = {left_motor, right_motor}
    # while laser_reading <= DIST_THESHOLD:
    #     robot.set_motors(STEERING_CONSTANT, -STEERING_CONSTANT)

def robotCalculations(data):
    mid_point = data.ranges[int(len(data.ranges)/2)]
    target_range = data.ranges[280:441]
    for dist in range(len(target_range)):
        if target_range[dist] < DIST_THESHOLD:
            laser_reading = target_range[dist]
            if dist <= mid_point:
                movement_pub.publish(turn_left_in_place(laser_reading))

            elif dist >= mid_point:
                movement_pub.publish(turn_right_in_place(laser_reading))

            else:
                movement_pub.publish(move_forward())
                


if __name__ == '__main__':
    rospy.init_node(CALCULATIONS_NODE_NAME, anonymous=False)
    rospy.Subscriber(LIDAR_TOPIC_NAME, LaserScan, robotCalculations)
    movement_pub = rospy.Publisher(STEERING_AND_THROTTLE_TOPIC_NAME, Motor, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()