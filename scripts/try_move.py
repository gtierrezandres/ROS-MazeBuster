#!/usr/bin/env python
import rospy
from tkinter import *
from std_msgs.msg import Float32

THROTTLE_TOPIC_NAME = "/throttle"
STEERING_TOPIC_NAME = "/steering"

rospy.init_node("Test_node", anonymous=False)
steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
rate = rospy.Rate(15)

def send_throttle(throttle_value):
    throttle_pub.publish(float(throttle_value))

def send_steering(steering_value):
    steering_pub.publish(float(steering_value))

def send_stop():
    print("STOP")
    throttle_pub.publish(0)
    steering_pub.publish(0)


master = Tk()
master2 = Tk()
global throttle_value, steering_value
throttle_slider = Scale(master, label = "throttle", from_=-0.5, to=0.5, resolution=0.01, command = send_throttle)
throttle_slider.set(0)
throttle_slider.pack()
steering_slider = Scale(master2, label = "steering", from_=-1, to=1, resolution=0.01, command = send_steering)
steering_slider.set(0)
steering_slider.pack()
Button(master, text='Stop', command=send_stop).pack()

mainloop()