#!/usr/bin/env python
from jetbot import Robot
import time
from sensor_msgs.msg import LaserScan
import rospy

def readings(data):
    mid_point = int(len(data.ranges)/2)
    target_range = data.ranges[480:600]
    for dist in range(len(target_range)):
        if target_range[dist] < 0.2:
            print(target_range[dist])
            # laser_reading = target_range[dist]
            # if dist <= mid_point:
            #     movement_pub.publish(turn_left_in_place(laser_reading))
            # elif dist >= mid_point:
            #     movement_pub.publish(turn_right_in_place(laser_reading))

        # else:
        #     movement_pub.publish(move_forward())

if __name__ == '__main__':
    rospy.init_node("scanning", anonymous=False)
    rospy.Subscriber("/scan", LaserScan, readings)
    while not rospy.is_shutdown():
        rospy.spin()