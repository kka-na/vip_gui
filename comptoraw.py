#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import subprocess, shlex, psutil
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def comptoraw(data) :
	np_arr = np.fromstring(data.data, np.uint8)
	cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	bridge = CvBridge()
	raw_img_pub.publish(bridge.cv2_to_imgmsg(cv2_img, "bgr8"))

def start_cam() :
	print("erp cam starting and converted from compressed image to raw image")
	command ="rosparam set /camera_erp/usb_cam/image_raw/compressed/jpeg_quality 10"
	command = shlex.split(command)
	subprocess.Popen(command)
	command ="roslaunch usb_cam usb_erp_cam.launch"
	command = shlex.split(command)
	subprocess.Popen(command)

start_cam()
raw_img_pub = rospy.Publisher("/camera_erp/converted/image_raw", Image, queue_size = 1)
rospy.init_node("visualizer", anonymous=True)
rospy.Subscriber('/camera_erp/usb_cam/image_raw/compressed',CompressedImage, comptoraw)
rospy.spin()
