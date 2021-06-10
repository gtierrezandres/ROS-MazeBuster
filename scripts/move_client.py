#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from jetbot import Robot
from jetbot import Heartbeat

MOVE_NODE_NAME = "move_client"
THROTTLE_TOPIC_NAME = "/throttle"
STEERING_TOPIC_NAME = "/steering"
MAX_THROTTLE = 0.5
MIN_THROTTLE = -0.5
MAX_STEERING = 1
MIN_STEERING = -1

rbt = Robot()
heartbeat = Heartbeat()

#----- Calibration values --------------
rbt.left_motor.alpha = 1
rbt.right_motor.alpha = 0.84
# rbt.left_motor.beta = 0.0
# rbt.right_motor.beta = 0.04
#----- End Calibration values ----------


def handle_hearbeat_status(change):
    if change['new'] == Heartbeat.Status.dead:
        rbt.stop()

def block(print):
    return

def throttle_handler(data):
    if data.data == 0:
        rbt.stop()
    if data.data > MAX_THROTTLE:
        data.data = MAX_THROTTLE
    if data.data < MIN_THROTTLE:
        data.data = MIN_THROTTLE
    direction = rbt.right_motor.value - rbt.left_motor.value
    if direction >= 0:  # Means the car turn left or not turning
        rbt.right_motor.value = data.data
        rbt.left_motor.value = data.data - abs(direction)
    else:               # Means the car turn right
        rbt.left_motor.value = data.data
        rbt.right_motor.value = data.data - abs(direction)
    print("LEFT: " + str(rbt.left_motor.value) + "      RIGHT: " + str(rbt.right_motor.value))


def steering_handler(data):
    if data.data > MAX_STEERING:
        data.data = MAX_STEERING
    if data.data < MIN_STEERING:
        data.data = MIN_STEERING
    speed = max(rbt.left_motor.value, rbt.right_motor.value)
    if data.data > 0: # Means turning right
        rbt.right_motor.value = speed
        rbt.left_motor.value = speed - abs(data.data)
    elif data.data == 0:
        rbt.right_motor.value = speed
        rbt.left_motor.value = speed
    else: # Means turning left
        rbt.right_motor.value = speed - abs(data.data)
        rbt.left_motor.value = speed
    print("LEFT: " + str(rbt.left_motor.value) + "      RIGHT: " + str(rbt.right_motor.value))

def listener():
    heartbeat.observe(handle_hearbeat_status, names='status')
    rospy.init_node(MOVE_NODE_NAME, anonymous=False)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, throttle_handler)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, steering_handler)
    rospy.spin()

if __name__ == '__main__':
    listener()
