#!/usr/bin/env python

import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import roslib
import sys
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
import argparse

from camera_info_manager import *

########################################################################################################
####################################### IP Address #####################################################
########################################################################################################

IP_address = '192.168.1.101'
GUI = False

########################################################################################################
#################################### IPCamera classs ###################################################
########################################################################################################

class IPCamera(object):
    def __init__(self, url):
        try:
            self.stream=urllib.urlopen(url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(url))
            sys.exit() #'Unable to open camera stream')
        self.bytes=''
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
        self.bridge = CvBridge()

########################################################################################################
######################################### Main #########################################################
########################################################################################################

if __name__ == '__main__':
    rospy.init_node('ip_camera_node', anonymous=True)
    ip_camera = IPCamera('http://'+ IP_address +':8080/video')
    camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)

    cinfo = CameraInfoManager(cname='camera', url='package://robot_april_detection/config/camera_info.yaml', namespace='camera')
    cinfo.loadCameraInfo()
    # print(type(cinfo.getCameraInfo()))

    while not rospy.is_shutdown():
        ip_camera.bytes += ip_camera.stream.read(1024)
        a = ip_camera.bytes.find('\xff\xd8')
        b = ip_camera.bytes.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = ip_camera.bytes[a:b+2]
            ip_camera.bytes= ip_camera.bytes[b+2:]
            if len(jpg) >0:
            	i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
	    image_message = i	
            ip_camera.image_pub.publish(ip_camera.bridge.cv2_to_imgmsg(image_message, "bgr8"))
            camera_info_pub.publish(cinfo.getCameraInfo())

	    if GUI:
               cv2.imshow('IP Camera Publisher Cam',i)
	    if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
	       exit(0) 
