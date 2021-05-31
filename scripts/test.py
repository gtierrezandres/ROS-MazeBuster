from jetbot import Robot
import time

def turn_in_place():
    robot = Robot()
    robot.left_motor.alpha = 1
    robot.right_motor.alpha = 0.84
    robot.set_motors(0.3, -0.3)
    time.sleep(1)

turn_in_place()