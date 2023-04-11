#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
cvb = CvBridge()
video = rospy.Publisher("/image/video/compressed",CompressedImage,queue_size=10)
rospy.init_node("video_topic")

while not rospy.is_shutdown():
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
        image = frame.array
        image = cv2.flip(image,0)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    #vid = cvb.cv2_to_compressed_imgmsg(image, encoding="passthrough")
        video.publish(msg)
    # show the frame
   #cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
    frame.release()
    cv2.destroyAllWindows()
