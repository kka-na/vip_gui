#!/usr/bin/env python
import rospy
import roslib
import sys, time
import math
import os
import cv2 
import threading
import numpy as np
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker

from djitellopy import Tello
#from pid import PID


tempx = 0.0
tempy = 0.0
tempyaw = 0.0
count = 1
#tello = Tello()
thread_running = False

def TelloStart(data) :
	global tello, tempx, tempy, tempyaw
	#tello.connect()
	#tello.streamon()
	#self.tello.takeoff()
	#self.tello.move_up(100) #20~500
	print("tello start")
	tempx = data.pose.pose.position.x
	tempy = data.pose.pose.position.y
	tempyaw = data.pose.pose.position.z
	
	img_thread = DroneTV()
	try:
		img_thread.daemon=True
		img_thread.start()
	except KeyboardInterrupt :
		img_thread.stop()
		print("Shut donw...")
	


def TelloControl(data) :
	global tello
	global tempx, tempy, tempyaw
	global count
	count += 1
	if(count%5 == 0) :
		x = data.pose.pose.position.x
		y = data.pose.pose.position.y
		yaw = data.twist.twist.angular.z
		distance = abs(math.hypot(x - tempx, y - tempy))
		rotate_degree = int(tempyaw-yaw) 
		#tello.rotate_clockwise(rotate_degree)
		#tello.move_forward(distance)
		print_drone = "-*-*-*-DroneTV State-*-*-*-\nX : {} Y : {} Distance : {} Yaw {}".format(x, y, distance, rotate_degree)
		print(print_drone)

		#Marker for Goal Point
		tello_drone = Marker()
		tello_drone.header.frame_id = "world"
		tello_drone.ns = "/tello_drone" #if user select goal
		tello_drone.type = tello_drone.CUBE
		tello_drone.action = tello_drone.ADD
		tello_drone.lifetime = rospy.Duration(0)
		tello_drone.id = 1
		tello_drone.pose.position.x = x
		tello_drone.pose.position.y = y
		tello_drone.pose.position.z = 20 #altitude
		tello_drone.pose.orientation.x = 0.0
		tello_drone.pose.orientation.y = 0.0
		tello_drone.pose.orientation.z = yaw #yaw
		tello_drone.pose.orientation.w = 1.0
		tello_drone.scale.x = 0.3
		tello_drone.scale.y = 0.3
		tello_drone.scale.z = 0.2
		#rgb(189, 147, 249) purple
		tello_drone.color.a = 1.0
		tello_drone.color.r = 0.74
		tello_drone.color.g = 0.58
		tello_drone.color.b = 0.97
		pub_tello_drone.publish(tello_drone)
		tempx = x
		tempy = y

	else :
		pass

class DroneTV(threading.Thread) :
	def __init__(self) :
		threading.Thread.__init__(self)
		global tello, thread_running
		#self.cap = tello.get_video_captuer()
		self.cap = cv2.VideoCapture(4) #now web cam
		thread_running = True


	def run(self) :
		global thread_running
		msg  = CompressedImage()
		while thread_running :
			b, frame = self.cap.read()
			if b:
				#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
				msg.header.stamp = rospy.Time.now()
				msg.format =  "jpeg"
				encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),10]
				msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tostring()
				pub_tello_img.publish(msg)
			else : 
				pass

	def stop(self) :
		thread_running = False
		self.quit()
		self.wait(5000)


pub_tello_drone = rospy.Publisher('/tello_drone', Marker, queue_size = 1, latch = True)
pub_tello_img = rospy.Publisher('/tello_img/image_raw/compressed', CompressedImage,  queue_size = 1)

rospy.init_node("visualizer",anonymous=True)

rospy.Subscriber('/start_pos', Odometry, TelloStart) 
rospy.Subscriber('/distance', Odometry, TelloControl)
rospy.spin()
