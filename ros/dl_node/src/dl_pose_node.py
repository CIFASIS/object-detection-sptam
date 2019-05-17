#!/usr/bin/env python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html




##import sys
##sys.path.append('/home/javier/ros_catkin_ws/install_isolated/lib/python2.7/site-packages')

import sys,time

# rospy for the subscriber and publisher
import roslib
import rospy
from rospy.numpy_msg import numpy_msg
# Ros Detection message
from dl_node.msg import DetectionWithPose
from dl_node.msg import DetectionWithPoseList

# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Header
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# SPTAM KF Message
from sptam.msg import StereoKFwithPose

# OpenCV2 for saving an image
print ('rostuff ok')
import cv2
print('cv2 ok')
import rcnn_pose as rcnn
print('rcnn ok')


class object_detector:
    
    def __init__(self):

        '''Initialize ros publisher, ros subscriber''' 
        det_topic = "/detection/stereo/left/det"     
   
        self.det_pub = rospy.Publisher(det_topic,DetectionWithPoseList,queue_size=10)
        print('detection publisher ok')

        ##image_topic = "/stereo/left/image" #Direct
        ##image_topic = "/keyframe/left/image_rect"
        keyframe_topic = "sptam/keyframe"
        # Set up your subscriber and define its callback
        self.im_sub = rospy.Subscriber(keyframe_topic, StereoKFwithPose, self.image_callback,queue_size=1)
 
        # Instantiate CvBridge
        self.bridge = CvBridge()
        
        print "Object Detect node ready"        

    def image_callback(self,msg):
        #print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg.img_l, "bgr8") # Probar rgb
            currentSeq = msg.header.seq
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            ##cv2.imwrite('camera_image.jpeg', cv2_img)
            detections,out = rcnn.getDetections(cv2_img,1)
         
            #### Create Message  ####
            det = DetectionWithPoseList()
            det.header = Header()
            det.header.stamp = rospy.Time.now()
            det.header.seq = currentSeq
            ##msg.layout.dim[0].label = "detection"
            
            det.data = detections
            ##det.pose = msg.pose ##Esto no va mas, ahora paso el pose por kf_id
            det.kf_id = msg.kf_id
            # Publish new image
            self.det_pub.publish(det)
               
            ##VER IMAGEN
            ##cv2.imshow('image',out)
            ##cv2.waitKey(1)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = object_detector()
    print 'Init Object Detector'
    rospy.init_node('object_detector')
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down ROS object detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv) 

