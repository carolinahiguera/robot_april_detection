#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import rospy

from camera_info_manager import CameraInfoManager

cap = cv2.VideoCapture(0)
bridge = CvBridge()
cinfo = CameraInfoManager(cname='camera', url='package://robot_april_detection/config/camera_info.yaml', namespace='camera')
cinfo.loadCameraInfo()

rospy.init_node('android_camera', anonymous=True)
image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)

while not rospy.is_shutdown():
    ret, frame_rgb = cap.read()
    if not ret:
        break
    image_pub.publish(bridge.cv2_to_imgmsg(frame_rgb, "bgr8"))
    camera_info_pub.publish(cinfo.getCameraInfo())
    # cv2.imshow('frame_RGB', frame_rgb)
    # k = cv2.waitKey(5) & 0xFF 
    # if k == 27:
    #     break

cv2.destroyAllWindows()
