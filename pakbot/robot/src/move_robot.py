#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String

rospy.init_node("Robot")

#robot = rospy.Publisher("/Robot_movement",String)
# Set up GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)

# Function to move the robot forward
def move_forward():
    GPIO.output(11, GPIO.HIGH)
    GPIO.output(13, GPIO.LOW)
    GPIO.output(15, GPIO.HIGH)
    GPIO.output(16, GPIO.LOW)

# Function to move the robot backward
def move_backward():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    GPIO.output(15, GPIO.LOW)
    GPIO.output(16, GPIO.HIGH)

# Function to turn the robot left
def turn_left():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    GPIO.output(15, GPIO.HIGH)
    GPIO.output(16, GPIO.LOW)

# Function to turn the robot right
def turn_right():
    GPIO.output(11, GPIO.HIGH)
    GPIO.output(13, GPIO.LOW)
    GPIO.output(15, GPIO.LOW)
    GPIO.output(16, GPIO.HIGH)

# Stop the robot
def stop():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)
    GPIO.output(15, GPIO.LOW)
    GPIO.output(16, GPIO.LOW)


def callback(data):
    ind = data.data
    if ind=="w":
        move_forward()
        time.sleep(0.05)
        stop()
    elif ind=="x":
        move_backward()
        time.sleep(0.05)
        stop()
    elif ind=="d":
        turn_left()
        time.sleep(0.05)
        stop()
    elif ind=="a":
        turn_right()
        time.sleep(0.05)
        stop()
    elif ind=="s":
        stop()

rospy.Subscriber("/move_robot",String,callback)
rospy.spin()

# Clean up GPIO pins
GPIO.cleanup()

