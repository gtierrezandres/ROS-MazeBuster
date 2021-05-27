#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from jetbot import Robot
from jetbot import Heartbeat

MOVE_NODE_NAME = "move_client"
MOVE_TOPIC_NAME = "/move"
MAX_THROTTLE = 0.8
rbt = Robot()
heartbeat = Heartbeat()

def handle_hearbeat_status(change):
    if change['new'] == Heartbeat.Status.dead:
        rbt.stop()

def callback(data):
    if data.data > MAX_THROTTLE:
        return
    if data.data == 0:
        rbt.stop()
    else:
        rbt.forward(data.data)

def listener():
    heartbeat.observe(handle_hearbeat_status, names='status')
    rospy.init_node(MOVE_NODE_NAME, anonymous=False)
    rospy.Subscriber(MOVE_TOPIC_NAME, Float32, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
