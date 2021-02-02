#! /usr/bin/env python
import sys
import copy
import tf
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import math
from time import sleep
import signal

import rospy
import rospkg
import roslib
import rviz
import actionlib

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5 import uic

#Get moving!
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PointStamped
from actionlib_msgs.msg import GoalID
# from pr2_controllers_msgs.msg import PointHeadActionGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal

#unique
from darknet_ros_msgs.msg import BoundingBoxes #darknet_ros

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu #drone-imu
from sensor_msgs.msg import NavSatFix #global-position
from sensor_msgs.msg import BatteryState #battery 
from std_msgs.msg import String
from geometry_msgs.msg import Point


#call ui file
form_class = uic.loadUiType("d2v.ui")[0]

class WindowClass(QMainWindow, form_class) :
	def __init__(self) : 
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.DCamThread = DCamThread()
		self.CCamThread = CCamThread()
		self.loadImageFromFile() #setting images
		self.actionReady.triggered.connect(self.actionready_triggered) #ready state
		self.actionStart.triggered.connect(self.actionstart_triggered) #start state
		self.DCamThread.update_cap.connect(self.d_cam_callback)
		self.CCamThread.update_cap.connect(self.c_cam_callback)
		self.actionStop.triggered.connect(self.actionstop_triggered) #stop state

	def actionready_triggered(self):
		print("Action Ready Triggered")
		self.initialize_all() #initialize rviz and ros nodes 

	def actionstart_triggered(self):
		print("Action Start Triggered")
		self.subscribe_nodes() #subscribing start
		#self.DCamThread.start()
		#self.CCamThread.start()

	def actionstop_triggered(self) :
		print("Action Stop Triggered")
		rospy.on_shutdown(self.myhook) #shut down the nodes
		self.DCamThread.stop()
		self.CCamThread.stop()

	def loadImageFromFile(self) :
		self.qPixmapCar = QPixmap()
		self.qPixmapCar.load("./icon/car.png")
		self.qPixmapCar = self.qPixmapCar.scaledToWidth(50)
		self.car_png.setPixmap(self.qPixmapCar)
		self.qPixmapDrone = QPixmap()
		self.qPixmapDrone.load("./icon/drone.png")
		self.qPixmapDrone = self.qPixmapDrone.scaledToWidth(50)
		self.drone_png.setPixmap(self.qPixmapDrone)

		self.qPixmapDynamic = QPixmap()
		self.qPixmapDynamic.load("./icon/person.png")
		self.qPixmapDynamic = self.qPixmapDynamic.scaledToWidth(50)
		self.c_static_png.setPixmap(self.qPixmapDynamic)
		self.d_static_png.setPixmap(self.qPixmapDynamic)
		self.qPixmapStatic = QPixmap()
		self.qPixmapStatic.load("./icon/cell_phone.png")
		#self.qPixmapDrone.load("./icon/cone.png")
		self.qPixmapStatic = self.qPixmapStatic.scaledToWidth(50)
		self.c_dynamic_png.setPixmap(self.qPixmapStatic)
		self.d_dynamic_png.setPixmap(self.qPixmapStatic)


	def initialize_all(self) :
		rospy.init_node('visualizer', anonymous=True); #set node 
		self.publish_nodes() #publish send NAV GOAL message from rviz 
		#setting rviz visulizer
		self.rviz_frame = rviz.VisualizationFrame()
		self.rviz_frame.setSplashPath("")
		self.rviz_frame.initialize()
		#setting rviz frame reader
		reader = rviz.YamlConfigReader()
		config = rviz.Config() 
		reader.readFile(config, "default.rviz") #this is saved rviz frame for d2v project
		#setting rviz tools 
		self.rviz_frame.load(config)
		self.rviz_frame.setMenuBar(None)
		self.rviz_frame.setStatusBar(None)
		#setting rviz manager
		self.manager = self.rviz_frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		self.rviz_layout.addWidget(self.rviz_frame)
		print("All Process were Initied")

	def publish_nodes(self) :
		#for sending nav goals
		nav_topic = rospy.get_param("remote_nav/nav_topic", "/move_base_simple/goal")
		self.nav_pub = rospy.Publisher(nav_topic, PoseStamped, queue_size = 10)

	#starting subscribe nodes
	def subscribe_nodes(self) :
		print("Nodes are Subscribing")
		#this is for car
		rospy.Subscriber('/goal_pos', Point, self.rviz_goal_callback) 
		rospy.Subscriber('/goal_node', String, self.rviz_goal_node_callback)
		rospy.Subscriber('/pose',Odometry, self.c_nav_callback) 
		rospy.Subscriber('/distance',Odometry, self.c_dist_callback) #!!!!!
		#rospy.Subscriber('/darknet_ros/detection_image_c', Image, self.c_cam_callback) #car
		rospy.Subscriber('/darknet_ros/bounding_boxes_c', BoundingBoxes, self.c_bboxes_callback)
		#rospy.Subscriber('/Vel', Odometry, self.c_vel_callback)
		#rospy.Subscriber('/obs_number',Obs, self.c_obs_callback ) #!!!!		

		#this is for drone
		rospy.Subscriber('/mavros/battery',BatteryState,self.d_batt_callback)
		rospy.Subscriber('/mavros/global_position/global',NavSatFix,self.d_pos_callback)
		rospy.Subscriber('/mavros/imu/data',Imu, self.d_imu_callback)
		rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.d_alt_callback)
		#rospy.Subscriber('/darknet_ros/detection_image_d', Image, self.d_cam_callback) #drone
		rospy.Subscriber('/darknet_ros/bounding_boxes_d',BoundingBoxes, self.d_bboxes_callback )


	"""
	def moveNav(self) :
		goal = PoseStamped()
		goal.header.frame_id = "/start"
		goal.pose.orientation.z = 1.0
		goal.pose.orientation.w = 0.0
		self._send_nav_goal(goal)

	def _send_nav_goal(self, pose) : 
		self.nav_pub.publish(pose)
	"""

	#unique message from rviz to display goal position information
	def rviz_goal_callback(self, data) :
		pos_txt = "X : {}   Y : {}".format(round(data.x,4), round(data.y,4))
		yaw_txt = "{}".format(round(data.z,4))
		self.goal_pos_label.setText(pos_txt)
		self.goal_yaw_label.setText(yaw_txt)

	#stdString message from rviz to dipslay nearest node's number
	def rviz_goal_node_callback(self, data) :
		self.goal_node_label.setText(data.data)
		
	#display car postion information 
	def c_nav_callback(self, ros_data) :
		pos_txt = "lat : {}   lng : {}".format(round(ros_data.pose.pose.position.y,4), round(ros_data.pose.pose.position.x,4))
		yaw_txt = "{}".format(round(ros_data.twist.twist.angular.z,4))
		self.car_pos_label.setText(pos_txt)
		self.car_yaw_label.setText(yaw_txt) #display yaw value on qlabel

	#display car velocity information
	def c_vel_callback(self, ros_data) :
		velstr = str(ros_data.pose.pose.position.x)
		self.car_vel_label.setText(velstr+" m/s") #modify dimension

	#display rest distance to destination 
	def c_dist_callback(self, ros_data) :
		diststr = str(ros_data.twist.twist.linear.z)
		self.car_dist_label.setText(diststr+" m remain") 

	#display obstacles number from LiDAR
	def c_obs_callback(self, ros_data) :
		obsstr = str(int(ros_data.pose.pose.position.x))
		self.car_obs_label.setText(obsstr+" detected") 

	#display object detection result from car
	@pyqtSlot(QImage)
 	def c_cam_callback(self, qimage) :
		self.car_cam_label_1.setPixmap(QPixmap(qimage))
		self.car_cam_label_1.show()
		QApplication.processEvents()
	"""
	def c_cam_callback(self, data) :
		self.bridge = CvBridge()
		try:
			cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print(e)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(480,360, Qt.KeepAspectRatio)
		self.car_cam_label_1.setPixmap(QPixmap.fromImage(qimage))

		self.car_cam_label_1.show()
		QApplication.processEvents()
	"""

	#display detectioned objects number from object detection
	def c_bboxes_callback(self, data) :
		s_count = 0
		d_count = 0
		for box in data.bounding_boxes:
			if box.Class == 'person' :
				s_count = s_count+1

			if box.Class == 'cell phone' :
				d_count = d_count+1

		self.c_static_num.setText(str(s_count))
		self.c_dynamic_num.setText(str(d_count))
		#self.c_static_png.hide()
		#self.c_dynamic_png.hide()

	#display drone's now position
	def d_pos_callback(self, data) :
		pos_txt = "lat : {}, lng : {}".format(data.latitude, data.longitude)
		self.drone_pos_label.setText(pos_txt)

	#display drone's IMU data
	def d_imu_callback(self, data) :
		quaternion = (data.orientation.x,data.orientation.y, data.orientation.z,data.orientation.w  )
		euler = tf.transformations.euler_from_quaternion(quaternion)	
		PI = 3.141592
		pitch = round(euler[0]*180/PI, 4)
		roll = round(euler[1]*180/PI, 4)
		yaw = round(euler[2]*180/PI, 4)

		ypr_txt = "Y:{}   P:{}  R:{}".format(yaw, pitch, roll)
		self.drone_ypr_label.setText(ypr_txt)

	#display drone's now altitude
	def d_alt_callback(self, data) :
		alt_txt = "{} m".format(data.pose.position.z)
		self.drone_alt_label.setText(alt_txt)

	#display drone's remain battery 
	def d_batt_callback(self, data) :
		bat_txt = "{} %".format(round(data.percentage*100,1))
		self.drone_batt_label.setText(bat_txt)

	#display object detection result from drone
	"""
	def d_cam_callback(self, data) :
		self.bridge = CvBridge()
		try:
			cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print(e)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(480,360, Qt.KeepAspectRatio)
		self.drone_cam_label.setPixmap(QPixmap.fromImage(qimage))

		self.drone_cam_label.show()
		QApplication.processEvents()
	"""
	@pyqtSlot(QImage)
 	def d_cam_callback(self, qimage) :
		self.drone_cam_label.setPixmap(QPixmap(qimage))
		self.drone_cam_label.show()
		QApplication.processEvents()

 	#diplay object's number from drones' object detection
	def d_bboxes_callback(self, data) :
		s_count = 0
		d_count = 0
		for box in data.bounding_boxes:
			if box.Class == 'person' :
				s_count = s_count+1
				
			if box.Class == 'cell phone' :
				d_count = d_count+1
				

		self.d_static_num.setText(str(s_count))
		self.d_dynamic_num.setText(str(d_count))
		#self.d_static_png.hide()
		#self.d_dynamic_png.hide()

	#shut down the ros nodes
	def myhook(self):
		print("Shut Down")


class CCamThread(QThread) :
	update_cap = pyqtSignal(QImage)
	def __init__(self) :
		super(CCamThread, self).__init__()
		self.bridge = CvBridge()
		self.img = rospy.Subscriber('/image_raw', Image, self.get_image)
		self.data = Image()
		self.cv2_img = np.zeros((480,360,3), np.uint8)
		self.running = True

	def get_image(self, data) :
		self.data = data

	def run(self) :
		while self.running :
			try:
				self.cv2_img = self.bridge.imgmsg_to_cv2(self.data, "bgr8")
			except CvBridgeError, e:
				print(e)
			rgbImage = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2RGB)
			h,w,ch = rgbImage.shape
			bpl = ch * w
			converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
			qimage = converToQtFormat.scaled(480,360, Qt.KeepAspectRatio)
			self.update_cap.emit(qimage)

	def stop(self) :
		self.running = False
		self.quit()
		self.wait(5000)


class DCamThread(QThread) :
	update_cap = pyqtSignal(QImage)
	def __init__(self) :
		super(DCamThread, self).__init__()
		self.bridge = CvBridge()
		self.img = rospy.Subscriber('/image_raw', Image, self.get_image)
		self.data = Image()
		self.cv2_img = np.zeros((480,360,3), np.uint8)
		self.running = True

	def get_image(self, data) :
		self.data = data

	def run(self) :
		while self.running :
			try:
				self.cv2_img = self.bridge.imgmsg_to_cv2(self.data, "bgr8")
			except CvBridgeError, e:
				print(e)

			rgbImage = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2RGB)
			h,w,ch = rgbImage.shape
			bpl = ch * w
			converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
			qimage = converToQtFormat.scaled(480,360, Qt.KeepAspectRatio)
			self.update_cap.emit(qimage)

	def stop(self) :
		self.running = False
		self.quit()
		self.wait(5000)


if __name__ == "__main__" :
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.showMaximized()

	app.exec_()

