#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# Set GPIO pin numbers
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
rospy.init_node("sensors")
usv = rospy.Publisher("/distance",Float32,queue_size=10)

def distance():
    # Send trigger signal
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.0000001)
    GPIO.output(GPIO_TRIGGER, False)

    # Record time when signal is sent and received
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # Calculate distance in centimeters
    time_elapsed = stop_time - start_time
    distance_cm = time_elapsed * 34300 / 2

    return distance_cm

while not rospy.is_shutdown():
    dist = distance()
    usv.publish(dist)
    time.sleep(0.5)

GPIO.cleanup()
