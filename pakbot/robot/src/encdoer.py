#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int64
rospy.init_node('encoder_publisher')  # Initialize the ROS node

#speed_msg = Vector3Stamped()  # Create a Vector3Stamped ROS message
#speed_pub = rospy.Publisher('/speed', Vector3Stamped, queue_size=10) 
encoder1 = rospy.Publisher("encoder1",Int64,queue_size=10)
encoder2 = rospy.Publisher("encoder2",Int64,queue_size=10)

# Encoder 1
encoder0PinA = 18
encoder0PinB = 19

# Encoder 2
encoder1PinA = 21
encoder1PinB = 20

encoder0Pos = 0
encoder1Pos = 0

motor1Pin1 = 11
motor1Pin2 = 13
motor2Pin1 = 15
motor2Pin2 = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(encoder0PinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder0PinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder1PinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder1PinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def doEncoderA(channel):
    global encoder0Pos
    if GPIO.input(encoder0PinA) == GPIO.HIGH:
        if GPIO.input(encoder0PinB) == GPIO.LOW:
            encoder0Pos += 1
        else:
            encoder0Pos -= 1
    else:
        if GPIO.input(encoder0PinB) == GPIO.HIGH:
            encoder0Pos += 1
        else:
            encoder0Pos -= 1

def doEncoderB(channel):
    global encoder0Pos
    if GPIO.input(encoder0PinB) == GPIO.HIGH:
        if GPIO.input(encoder0PinA) == GPIO.HIGH:
            encoder0Pos += 1
        else:
            encoder0Pos -= 1
    else:
        if GPIO.input(encoder0PinA) == GPIO.LOW:
            encoder0Pos += 1
        else:
            encoder0Pos -= 1

def doEncoderC(channel):
    global encoder1Pos
    if GPIO.input(encoder1PinA) == GPIO.HIGH:
        if GPIO.input(encoder1PinB) == GPIO.LOW:
            encoder1Pos -= 1
        else:
            encoder1Pos += 1
    else:
        if GPIO.input(encoder1PinB) == GPIO.HIGH:
            encoder1Pos -= 1
        else:
            encoder1Pos += 1

def doEncoderD(channel):
    global encoder1Pos
    if GPIO.input(encoder1PinB) == GPIO.HIGH:
        if GPIO.input(encoder1PinA) == GPIO.HIGH:
            encoder1Pos -= 1
        else:
            encoder1Pos += 1
    else:
        if GPIO.input(encoder1PinA) == GPIO.LOW:
            encoder1Pos -= 1
        else:
            encoder1Pos += 1


GPIO.add_event_detect(encoder0PinA, GPIO.BOTH, callback=doEncoderA)
GPIO.add_event_detect(encoder0PinB, GPIO.BOTH, callback=doEncoderB)
GPIO.add_event_detect(encoder1PinA, GPIO.BOTH, callback=doEncoderC)
GPIO.add_event_detect(encoder1PinB, GPIO.BOTH, callback=doEncoderD)

rate = rospy.Rate(10)

previousMillis = int(time.time() * 1000)
while not rospy.is_shutdown():
    currentMillis = int(time.time() * 1000)
    if currentMillis - previousMillis >= 10:
        previousMillis = currentMillis
        print(encoder0Pos,encoder1Pos)
        #encoder0Prev,encoder1Prev=0,0
        time.sleep(0.05)
        encoder1.publish(encoder0Pos)
        encoder2.publish(encoder1Pos)
    rate.sleep()

GPIO.cleanup()

