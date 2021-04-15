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
import os
import subprocess, shlex, psutil
import csv 

import rospy
import rospkg
import roslib
import rosbag
import rviz
import actionlib
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

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

from sensor_msgs.msg import Image,CompressedImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu #drone-imu
from sensor_msgs.msg import NavSatFix #global-position
from sensor_msgs.msg import BatteryState #battery 
from std_msgs.msg import String
from geometry_msgs.msg import Point


#call ui file
form_class = uic.loadUiType("d2v.ui")[0]

playbar_start = 0
playbar_temp = 0
playbar_end = 0
play_btn_clicked = False
pause_btn_clicked = False

class WindowClass(QMainWindow, form_class) :
	def __init__(self) : 
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.rosbag_proc = subprocess
		rospy.init_node('visualizer', anonymous=True)
		self.loadImageFromFile() #setting images
		self.actionReady.triggered.connect(self.actionready_triggered) #ready state
		self.actionReady_2.triggered.connect(self.actionready_2_triggered)
		self.actionRecord.triggered.connect(self.actionrecord_triggered)
		self.actionStart.triggered.connect(self.actionstart_triggered) #start state
		self.actionStart_2.triggered.connect(self.actionstart_2_triggered) #start state
		self.actionStop.triggered.connect(self.actionstop_triggered) #stop state

	def actionready_triggered(self):
		print("Action Ready Triggered")
		self.initialize_all() #initialize rviz and ros nodes

	def actionrecord_triggered(self):
		print("Action Record Triggered")
		self.set_record()

	def actionstart_triggered(self):
		print("Action Start Triggered")
		self.subscribe_nodes() #subscribing start

	def actionstop_triggered(self) :
		print("Action Stop Triggered")
		self.monitoring_stop()


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
		#self.d_static_png.setPixmap(self.qPixmapDynamic)
		self.qPixmapStatic = QPixmap()
		self.qPixmapStatic.load("./icon/cone.png")
		self.qPixmapStatic = self.qPixmapStatic.scaledToWidth(50)
		self.qPixmapStatic_2 = QPixmap()
		self.qPixmapStatic_2.load("./icon/banana.png")
		self.qPixmapStatic_2 = self.qPixmapStatic_2.scaledToWidth(50)
		self.c_dynamic_png.setPixmap(self.qPixmapStatic)
		#self.d_dynamic_png.setPixmap(self.qPixmapStatic_2)
		self.label_18.hide()


	def initialize_all(self) :
		#self.publish_nodes() #publish send NAV GOAL message from rviz 
		#setting rviz visulizer
		self.rviz_frame = rviz.VisualizationFrame()
		self.rviz_frame.setSplashPath("")
		self.rviz_frame.initialize()
		#setting rviz frame reader
		reader = rviz.YamlConfigReader()
		config = rviz.Config() 
		reader.readFile(config, "./default.rviz") #this is saved rviz frame for d2v project
		#setting rviz tools 
		self.rviz_frame.load(config)
		self.rviz_frame.setMenuBar(None)
		self.rviz_frame.setStatusBar(None)
		#setting rviz manager
		self.manager = self.rviz_frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		self.rviz_layout.addWidget(self.rviz_frame)

		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Ready State")
		mbox.setText("All Process were Initialized     ")
		mbox.exec_()

	def set_record(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Record State")
		mbox.setText("All information will be recorded.     \nWould you like to record it?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		returnv = mbox.exec_()
		path = ""
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save recorded information',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			command ="rosbag record -o "+path+"/ -a"
			command = shlex.split(command)
			self.rosbag_proc = subprocess.Popen(command)

	#starting subscribe nodes
	def subscribe_nodes(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Start State")
		mbox.setText("Monitoring Start!     ")
		mbox.exec_()

		#this is for goal
		rospy.Subscriber('/goal_pos', Point, self.rviz_goal_callback) 
		rospy.Subscriber('/goal_node', String, self.rviz_goal_node_callback)

		#this is for car
		rospy.Subscriber('/pose',Odometry, self.c_nav_callback) 
		rospy.Subscriber('/distance',Odometry, self.c_dist_callback) #!!!!!
		rospy.Subscriber('/lane_detection/image_raw/compressed', CompressedImage, self.c_lane_cam_callback)
		rospy.Subscriber('/darknet_ros/detection_image_c/compressed', CompressedImage, self.c_cam_callback) #car
		rospy.Subscriber('/darknet_ros/bounding_boxes_c', BoundingBoxes, self.c_bboxes_callback)
		#rospy.Subscriber('/Vel', Odometry, self.c_vel_callback)
		rospy.Subscriber('/obstacle_number', Odometry, self.c_obs_callback ) #!!!!		

		#this is for drone
		rospy.Subscriber('/mavros/battery',BatteryState,self.d_batt_callback)
		rospy.Subscriber('/mavros/global_position/global',NavSatFix,self.d_pos_callback)
		rospy.Subscriber('/mavros/imu/data',Imu, self.d_imu_callback)
		rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.d_alt_callback)
		rospy.Subscriber('/camera_nano/object_detect/image_raw/compressed', CompressedImage, self.d_cam_callback) #drone
		rospy.Subscriber('/warning_msg', String, self.warning_callback)


	#unique message from rviz to display goal position information
	def rviz_goal_callback(self, data) :
		pos_txt = "X : {}   Y : {}".format(round(data.x,4), round(data.y,4))
		self.goal_pos_label.setText(pos_txt)
		yaw_txt = "{}".format(round(data.z,4))
		self.goal_yaw_label.setText(yaw_txt)

	#stdString message from rviz to dipslay nearest node's number
	def rviz_goal_node_callback(self, data) :
		self.goal_node_label.setText(data.data)
		
	#display car postion information 
	def c_nav_callback(self, ros_data) :
		pos_txt = "lat : {}   lng : {}".format(round(ros_data.pose.pose.position.y,4), round(ros_data.pose.pose.position.x,4))
		self.car_pos_label.setText(pos_txt)
		yaw_txt = "{}".format(round(ros_data.twist.twist.angular.z,4))
		self.car_yaw_label.setText(yaw_txt) #display yaw value on qlabel

	#display car velocity information
	def c_vel_callback(self, ros_data) :
		velstr = str(ros_data.pose.pose.position.x)
		self.car_vel_label.setText(velstr+" m/s") #modify dimension

	#display rest distance to destination 
	def c_dist_callback(self, ros_data) :
		diststr = str(ros_data.twist.twist.linear.z)
		self.car_distance_label.setText(diststr+" m remain") 

	#display obstacles number from LiDAR
	def c_obs_callback(self, ros_data) :
		obsstr = str(int(ros_data.pose.pose.position.x))
		self.car_obs_label.setText(obsstr+" detected") 

	#display object detection result from car
	def c_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(400,300, Qt.KeepAspectRatio)
		self.car_cam_label_1.setPixmap(QPixmap.fromImage(qimage))

		self.car_cam_label_1.show()
		QApplication.processEvents()

	#display object detection result from car
	def c_lane_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(480,300, Qt.KeepAspectRatio)
		self.car_cam_label_2.setPixmap(QPixmap.fromImage(qimage))
		self.car_cam_label_2.show()
		QApplication.processEvents()

	#display detectioned objects number from object detection
	def c_bboxes_callback(self, data) :
		s_count = 0
		d_count = 0
		for box in data.bounding_boxes:
			if box.Class == 'PERSON' :
				s_count = s_count+1

			if box.Class == 'CONE' :
				d_count = d_count+1

		self.c_static_num.setText(str(s_count))
		self.c_dynamic_num.setText(str(d_count))


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

	#display fly-cctv result from drone
	def d_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(480,300, Qt.KeepAspectRatio)
		self.drone_cam_label.setPixmap(QPixmap.fromImage(qimage))
		self.drone_cam_label.show()
		QApplication.processEvents()

	def warning_callback(self,data) :
		self.label_18.setText(data.data)
		self.label_18.show()
		sleep(0.2)
		self.label_18.hide()


	def monitoring_stop(self) :
		for proc in psutil.process_iter() :
			if "record" in proc.name() : 
				proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		rospy.on_shutdown(self.myhook) #shut down the nodes
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Stop State")
		mbox.setText("All Process were Ended     ")
		mbox.exec_()

	#shut down the ros nodes
	def myhook(self):
		print("Shut down all processes cleary!")

# ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ - End of Monitoring Processs - ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ #

	def actionready_2_triggered(self):
		print("Action Ready_2 Triggered")
		self.PlayBarThread = PlayBarThread()
		self.PlayBarThread.change_playbar_value.connect(self.change_playbar_state)
		self.setting_playback()
		self.initialize_all()

	def setting_playback(self) :
		self.bagfile =  QFileDialog.getOpenFileName(self, 'Select .bag files to playback', './')[0]
		self.bag = rosbag.Bag(self.bagfile)
		b_start = self.bag.get_start_time()
		b_end = self.bag.get_end_time()
		self.b_total_sec = int(b_end-b_start)

		self.slider.setRange(0, self.b_total_sec)
		self.slider.valueChanged.connect(self.setBagPosition)
		self.play_btn.clicked.connect(self.push_play_btn)
		self.pause_btn.clicked.connect(self.push_pause_btn)

	def actionstart_2_triggered(self) :
		global playbar_start, playbar_end
		playbar_start = 0
		playbar_end = self.b_total_sec
		self.subscribe_nodes() #subscribing start
		command ="rosbag play "+self.bagfile
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		self.PlayBarThread.start()


	def setBagPosition(self) :
		global playbar_start
		playbar_start = self.slider.value()
		self.ts_txt.setText(str(self.slider.value()))

	def push_play_btn(self) :
		global playbar_start
		global play_btn_clicked
		print("play btn pushed")
		play_btn_clicked = True
		command ="rosbag play -s"+str(playbar_start)+" "+self.bagfile
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		self.PlayBarThread.start()

	def push_pause_btn(self) :
		global pause_btn_clicked
		print("pause btn pushed")
		pause_btn_clicked = True
		for proc in psutil.process_iter() :
			if "play" in proc.name() :
				proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		self.PlayBarThread.stop()


	@pyqtSlot(int)
	def change_playbar_state(self, value) :
		self.slider.setValue(value)
		self.ts_txt.setText(str(value))



class PlayBarThread(QThread) :
	change_playbar_value = pyqtSignal(int)
	def __init__(self) :
		super(PlayBarThread, self).__init__()
		self.running = True

	def run(self) :
		global playbar_start, playbar_end, playbar_temp
		global play_btn_clicked, pause_btn_clicked
		if pause_btn_clicked :
			b_cnt = playbar_temp
			pause_btn_clicked = False
		else :
			b_cnt = playbar_start
		if play_btn_clicked :
			self.running = True

		while b_cnt in range(playbar_start, playbar_end+1) :
			if self.running == False : break
			if(b_cnt > playbar_end) :
				self.stop()
			self.change_playbar_value.emit(b_cnt)
			QApplication.processEvents()
			sleep(1)
			playbar_temp = b_cnt
			b_cnt += 1

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
