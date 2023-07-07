#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
rospy.init_node("Robot")

#robot = rospy.Publisher("/Robot_movement",String)
# Set up GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)

twist = Twist()

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


def callback(twist):
    print("Hi")
    linear_velocity = twist.linear.x
    angular_velocity = twist.angular.z
    print(linear_velocity,angular_velocity)
    if linear_velocity>0:
        move_forward()
        time.sleep(0.5)
        stop()
    elif linear_velocity<0:
        move_backward()
        time.sleep(0.5)
        stop()
    elif angular_velocity<0:
        turn_left()
        time.sleep(0.5)
        stop()
    elif angular_velocity>0:
        turn_right()
        time.sleep(0.5)
        stop()
    elif linear_velocity == 0 and angular_velocity ==0:
        stop()
    twist=0

#twist = Twist()
rospy.Subscriber("/cmd_vel",Twist,callback)
rospy.spin()

# Clean up GPIO pins
GPIO.cleanup()

