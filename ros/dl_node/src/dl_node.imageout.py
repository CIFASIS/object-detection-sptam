#!/usr/bin/env python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html




##import sys
##sys.path.append('/home/javier/ros_catkin_ws/install_isolated/lib/python2.7/site-packages')


# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
print ('rostuff ok')
import cv2
print('cv2 ok')
import rcnn as rcnn
print('rcnn ok')

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2_img = cv2.imread('/home/javier/rcnn/py-faster-rcnn/data/modelnet_demo/real02.jpg') 
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        ##cv2.imwrite('camera_image.jpeg', cv2_img)
        #detections = rcnn.getDetections(cv2_img)
        
        out = rcnn.demo(cv2_img)
        cv2.imshow('image',out)
        cv2.waitKey(1)

def listener():
    rospy.init_node('image_listener')
    # Define your image topic
    #image_topic = "/keyframe/left/image_rect" #Sptam
    image_topic = "/stereo/left/image" #Direct
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    listener()
