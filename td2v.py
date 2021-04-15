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
form_class = uic.loadUiType("td2v.ui")[0]
dialog_class = uic.loadUiType("dialogwidget.ui")[0]

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
		self.tdia = TopicDialog()
		self.loadImageFromFile() #setting images
		self.actionReady.triggered.connect(self.actionready_triggered) #ready state
		self.actionReady_2.triggered.connect(self.actionready_2_triggered)
		self.actionRecord.triggered.connect(self.actionrecord_triggered)
		self.actionStart.triggered.connect(self.actionstart_triggered) #start state
		self.actionStart_2.triggered.connect(self.actionstart_2_triggered) #start state
		self.actionStop.triggered.connect(self.actionstop_triggered) #stop state

	def actionready_triggered(self):
		print("Action Ready Triggered")
		self.topic_list_create()
		self.initialize_all() #initialize rviz and ros nodes

	def actionrecord_triggered(self):
		print("Action Record Triggered")
		self.set_record()

	def actionstart_triggered(self):
		print("Action Start Triggered")
		self.get_topic_list()
		#self.subscribe_nodes() #subscribing start

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


	def topic_list_create(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("ROS Topic List Create")
		mbox.setText("Create a topic to use.     \nCreate or Not?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		buttonC = mbox.button(QMessageBox.Ok)
		buttonC.setText('Create')
		returnv = mbox.exec_()
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save topic setting information csv',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			self.tdia.save_topics_csv(path)
			self.tdia.exec_()


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

	def publish_nodes(self) :
		#for sending nav goals
		nav_topic = rospy.get_param("remote_nav/nav_topic", "/move_base_simple/goal")
		self.nav_pub = rospy.Publisher(nav_topic, PoseStamped, queue_size=10)

	def get_topic_list(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("ROS Topic Setting")
		mbox.setText("Import topic list files to Use     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		buttonC = mbox.button(QMessageBox.Ok)
		buttonC.setText('Import')
		returnv = mbox.exec_()
		if returnv == QMessageBox.Ok :
			file =  QFileDialog.getOpenFileName(None, 'Select topic setting information csv files to setting ROS','./')[0]
			self.get_topics_csv(file)
			
	def get_topics_csv(self, file) :
		f = open(str(file), 'r')
		rdr = csv.reader(f)
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Start State")
		mbox.setText("Monitoring Start!     ")
		mbox.exec_()

		for line in rdr :
			self.initialize_ros(line[0], line[1], line[2])

	def initialize_ros(self, frame_name, topic_name, msg_type) : 
		self.testt()
		if(msg_type == "Odometry") :
			if(frame_name == "erp_pos") :
				rospy.Subscriber(topic_name, Odometry , self.c_pos_callback)
			if(frame_name == "erp_yaw") :
				rospy.Subscriber(topic_name, Odometry , self.c_yaw_callback)
			if(frame_name == "erp_dist") :
				rospy.Subscriber(topic_name, Odometry , self.c_dist_callback)
			if(frame_name == "obstacle_number") :
				rospy.Subscriber(topic_name, Odometry , self.c_obs_callback)
		if(msg_type == "CompressedImage") :
			if(frame_name == "erp_cam01") :
				rospy.Subscriber(topic_name, CompressedImage , self.c_cam_callback)
			if(frame_name == "erp_cam02") :
				rospy.Subscriber(topic_name, CompressedImage , self.c_lane_cam_callback)
			if(frame_name == "dro_cam01") :
				rospy.Subscriber(topic_name, CompressedImage, self.d_cam_callback)

		if(msg_type == "BoundingBoxes") :
			if(frame_name == "erp_detect") :
				rospy.Subscriber(topic_name, BoundingBoxes , self.c_bboxes_callback)
		if(msg_type == "BatteryState") :
			if(frame_name == "dro_bat") :
				rospy.Subscriber(topic_name, BatteryState , self.d_batt_callback)
		if(msg_type == "NavSatFix") :
			if(frame_name == "dro_pos") :
				rospy.Subscriber(topic_name, NavSatFix, self.d_pos_callback)
		if(msg_type == "Imu") :
			if(frame_name == "dro_ypr") :
				rospy.Subscriber(topic_name, Imu, self.d_imu_callback)
		if(msg_type == "PoseStamped") :
			if(frame_name == "dro_alt") :
				rospy.Subscriber(topic_name, PoseStamped, self.d_alt_callback)

		if(msg_type == "Point") :
			if(frame_name == "goal_pos") :
				rospy.Subscriber(topic_name, Point, self.rviz_goal_pos_callback)
			if(frame_name == "goal_head") :
				rospy.Subscriber(topic_name, Point, self.rviz_goal_head_callback)
		if(msg_type == "String") :
			if(frame_name == "goal_node") :
				rospy.Subscriber(topic_name, String, self.rviz_goal_node_callback)

		rospy.Subscriber('/warning_msg', String, self.warning_callback)

	"""
	#starting subscribe nodes
	def subscribe_nodes(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Start State")
		mbox.setText("Monitoring Start!     ")
		mbox.exec_()

		#this is for car
		rospy.Subscriber('/goal_pos', Point, self.rviz_goal_callback) 
		rospy.Subscriber('/goal_node', String, self.rviz_goal_node_callback)
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
		#rospy.Subscriber('/darknet_ros/bounding_boxes_d', BoundingBoxes, self.d_bboxes_callback) #drone
		#rospy.Subscriber('/camera_nano/usb_cam/image_raw/compressed', CompressedImage, self.d_cam_callback) #drone
		rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.d_cam_callback) #drone
	"""

	def moveNav(self) :
		goal = PoseStamped()
		goal.header.frame_id = "/start"
		goal.pose.orientation.z = 1.0
		goal.pose.orientation.w = 0.0
		self._send_nav_goal(goal)

	def _send_nav_goal(self, pose) : 
		self.nav_pub.publish(pose)

	def testt(self):
		pos_txt = " X : 6.3712 Y : 53.9518"
		self.goal_pos_label.setText(pos_txt)
		yaw_txt = "0.0"
		self.goal_yaw_label.setText(yaw_txt)
		self.goal_node_label.setText("21")

	#unique message from rviz to display goal position information
	def rviz_goal_pos_callback(self, data) :
		#pos_txt = "X : {}   Y : {}".format(round(data.x,4), round(data.y,4))
		pos_txt = " X : 6.3712 Y : 53.9518"
		self.goal_pos_label.setText(pos_txt)

	def rviz_goal_head_callback(self, data) :
		#yaw_txt = "{}".format(round(data.z,4))
		yaw_txt = "0.0"
		self.goal_yaw_label.setText(yaw_txt)

	#stdString message from rviz to dipslay nearest node's number
	def rviz_goal_node_callback(self, data) :
		#data.data
		self.goal_node_label.setText("21")
		
	#display car postion information 
	def c_pos_callback(self, ros_data) :
		pos_txt = "lat : {}   lng : {}".format(round(ros_data.pose.pose.position.y,4), round(ros_data.pose.pose.position.x,4))
		self.car_pos_label.setText(pos_txt)

	#display car postion information 
	def c_yaw_callback(self, ros_data) :
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
		bat_txt = "{} v".format(round(data.voltage,3))
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
		sleep(0.05)
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



class TopicDialog(QDialog, dialog_class) :
	def __init__(self) :
		super(TopicDialog, self).__init__()
		self.setupUi(self)
		self.buttonBox.accepted.connect(self.push_ok)

	def save_topics_csv(self, path) :
		file, ok = QInputDialog.getText(self, 'File name', 'Enter topic lists file\'s name')
		self.will_save = " "
		if ok :
			self.will_save = str(path) +"/"+ str(file)+".csv"
			print(self.will_save)

	def push_ok(self) :
		#~erp 
		erp_pos_n = str(self.erp_pos_n.toPlainText())
		erp_pos_t = str(self.erp_pos_t.currentText())
		erp_vel_n = str(self.erp_vel_n.toPlainText())
		erp_vel_t = str(self.erp_vel_t.currentText())
		erp_yaw_n = str(self.erp_yaw_n.toPlainText())
		erp_yaw_t = str(self.erp_yaw_t.currentText())
		erp_dist_n = str(self.erp_dist_n.toPlainText())
		erp_dist_t = str(self.erp_dist_t.currentText())
		erp_obs_n = str(self.erp_obs_n.toPlainText())
		erp_obs_t = str(self.erp_obs_t.currentText())
		erp_detect_n = str(self.erp_detect_n.toPlainText())
		erp_detect_t = str(self.erp_detect_t.currentText())
		erp_cam01_n = str(self.erp_cam01_n.toPlainText())
		erp_cam01_t = str(self.erp_cam01_t.currentText())
		erp_cam02_n = str(self.erp_cam02_n.toPlainText())
		erp_cam02_t = str(self.erp_cam02_t.currentText())
		#~drone
		dro_pos_n = str(self.dro_pos_n.toPlainText())
		dro_pos_t = str(self.dro_pos_t.currentText())
		dro_ypr_n = str(self.dro_ypr_n.toPlainText())
		dro_ypr_t = str(self.dro_ypr_t.currentText())
		dro_alt_n = str(self.dro_alt_n.toPlainText())
		dro_alt_t = str(self.dro_alt_t.currentText())
		dro_bat_n = str(self.dro_bat_n.toPlainText())
		dro_bat_t = str(self.dro_bat_t.currentText())
		dro_cam01_n = str(self.dro_cam01_n.toPlainText())
		dro_cam01_t = str(self.dro_cam01_t.currentText())

		#~goal
		goal_pos_n = str(self.goal_pos_n.toPlainText())
		goal_pos_t = str(self.goal_pos_t.currentText())
		goal_head_n = str(self.goal_head_n.toPlainText())
		goal_head_t = str(self.goal_head_t.currentText())
		goal_node_n = str(self.goal_node_n.toPlainText())
		goal_node_t = str(self.goal_node_t.currentText())

		f = open(self.will_save, 'w')
		wr = csv.writer(f)
		wr.writerow(['erp_pos', erp_pos_n, erp_pos_t])
		wr.writerow(['erp_vel', erp_vel_n, erp_vel_t])
		wr.writerow(['erp_yaw', erp_yaw_n, erp_yaw_t])
		wr.writerow(['erp_dist', erp_dist_n, erp_dist_t])
		wr.writerow(['erp_obs', erp_obs_n, erp_obs_t])
		wr.writerow(['erp_detect', erp_detect_n, erp_detect_t])
		wr.writerow(['erp_cam01', erp_cam01_n, erp_cam01_t])
		wr.writerow(['erp_cam02', erp_cam02_n, erp_cam02_t])
		wr.writerow(['dro_pos', dro_pos_n, dro_pos_t])
		wr.writerow(['dro_ypr', dro_ypr_n, dro_ypr_t])
		wr.writerow(['dro_alt', dro_alt_n, dro_alt_t])
		wr.writerow(['dro_bat', dro_bat_n, dro_bat_t])
		wr.writerow(['dro_cam01', dro_cam01_n, dro_cam01_t])
		wr.writerow(['goal_pos', goal_pos_n, goal_pos_t])
		wr.writerow(['goal_head', goal_head_n, goal_head_t])
		wr.writerow(['goal_node', goal_node_n, goal_node_t])

		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Topic List Create")
		mbox.setText("A new topic list csv file was created     ")
		mbox.exec_()


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


"""


		self.update_topics.emit('erp_pos', erp_pos_n, erp_pos_t)
		self.update_topics.emit('erp_vel', erp_vel_n, erp_vel_t)
		self.update_topics.emit('erp_yaw', erp_yaw_n, erp_yaw_t)
		self.update_topics.emit('erp_dist', erp_dist_n, erp_dist_t)
		self.update_topics.emit('erp_obs', erp_obs_n, erp_obs_t)
		self.update_topics.emit('erp_detect', erp_detect_n, erp_detect_t)
		self.update_topics.emit('erp_cam01', erp_cam01_n, erp_cam01_t)
		self.update_topics.emit('erp_cam02', erp_cam02_n, erp_cam02_t)
		self.update_topics.emit('dro_pos', dro_pos_n, dro_pos_t)
		self.update_topics.emit('dro_ypr', dro_ypr_n, dro_ypr_t)
		self.update_topics.emit('dro_alt', dro_alt_n, dro_alt_t)
		self.update_topics.emit('dro_bat', dro_bat_n, dro_bat_t)
		self.update_topics.emit('goal_pos', goal_pos_n, goal_pos_t)
		self.update_topics.emit('goal_head', goal_head_n, goal_head_t)
		self.update_topics.emit('goal_node', goal_node_n, goal_node_t)

		"""