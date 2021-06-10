#!/usr/bin/env python
import jetbot
import rospy
from std_msgs.msg import Float32
from maze_buster.msg import Motor
import pygame

STEERING_AND_THROTTLE_TOPIC_NAME = 'cmd_vel'
MANUAL_DRIVE_NODE_NAME = 'manual_drive_node'
THROTTLE_CONSTANT = 0.5
STEERING_CONSTANT = 0.5

def move_forward():
    left_motor.data = THROTTLE_CONSTANT
    right_motor.data = THROTTLE_CONSTANT
    motors = [left_motor.data, right_motor.data]
    return motors

def turn_right_in_place():
    left_motor.data = STEERING_CONSTANT
    right_motor.data = -STEERING_CONSTANT
    motors = [left_motor.data, right_motor.data]
    return motors

def turn_left_in_place(laser_reading):
    left_motor.data = -STEERING_CONSTANT
    right_motor.data = STEERING_CONSTANT
    motors = [left_motor.data, right_motor.data]
    return motors

def manual_mode():
    running = True
    while running:
        pygame.init()
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    manual_mv_pub.publish(move_forward())
                if event.key == pygame.K_d:
                    manual_mv_pub.publish(turn_right_in_place())
                if event.key == pygame.K_a:
                    manual_mv_pub.publish(turn_left_in_place())
        if rospy.is_shutdown == True:
            pygame.display.quit()
            running = False
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    left_motor = Float32()
    left_motor.data = 0.0

    right_motor = Float32()
    right_motor.data = 0.0

    motors = Motor()

    rospy.init_node(MANUAL_DRIVE_NODE_NAME, anonymous=False)
    manual_mv_pub = rospy.Publisher(STEERING_AND_THROTTLE_TOPIC_NAME, Motor, queue_size=1)
    rate = rospy.Rate(15)
    manual_mode()