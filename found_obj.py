#!/usr/bin/env python
import rospy
import numpy as np
import time
import math
import tf
import pymap3d
import csv
import io
import os, glob
import os.path
import subprocess, shlex, psutil

from darknet_ros_msgs.msg import BoundingBoxes #darknet_ros
from sensor_msgs.msg import NavSatFix #global-position
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

"""
BoundingBox.msg
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 id
string Class
"""
goal_sent_successfully = False
darknet_ros_proc = ""
drone_object_detection_status = False
find_count = 0

def start_drone_object_detection() :
	global darknet_ros_proc
	global drone_object_detection_status
	print("drone object detection started")
	drone_object_detection_status = True
	command ="roslaunch darknet_ros darknet_ros_drone.launch"
	command = shlex.split(command)
	darknet_ros_proc = subprocess.Popen(command)

def objConditionCheck(topic) :
	global goal_sent_successfully
	global find_count
	condition = 320 #specific condition 
	for box in topic.bounding_boxes:
		if box.Class == 'banana' :
			diagonal = abs(math.hypot(box.xmax-box.xmin, box.ymax - box.ymin))
			print(str(diagonal))
			if diagonal >= condition :
				find_count += 1
				if find_count >= 10 :
					if goal_sent_successfully == False :
						end_drone_obj_detection()
						#rospy.Subscriber('/mavros/global_position/global',NavSatFix, findgoal)
					else :
						break
				else :
					pass
			else :
				pass
		else :
			pass


def findgoal(data): 
	global goal_sent_successfully
	# base point offset parameter
	base_lat = 37.383784
	base_lon = 126.654310
	base_alt = 30.3

	#get the lat, lng from drone 
	lat = float(data.latitude)
	lng = float(data.longitude)
	alt = 34.5

	# latitude, longitude  -->  x, y use pymap3d
	x0, y0, u0 = pymap3d.geodetic2enu(lat, lng, alt, base_lat, base_lon, base_alt)

	#initialize
	result_x = 0.0
	result_y = 0.0
	min_dis = 9999.9
	x1 = 0.0
	y1 = 0.0
	x2 = x0 #x position from banana position that drone detected
	y2 = y0 #y position from banana position that drone detected

	#list of map csv files 
	target_dir = "/home/kanakim/catkin_ws/src/gui/map" #have to change own csv map folder
	files = os.listdir(target_dir)

	#initialize result position object
	position_result = np.empty([0,3], int)
	line_files = []

	files_idx = -1
	check_idx = 0

	#read the all csv files
	for number in files:
		file = "/home/kanakim/catkin_ws/src/gui/map/" + number #have to change own csv map folder
		f = io.open(file, 'r', encoding='utf-8')
		rdr = csv.reader(f)

		line_files.append(number)
		files_idx += 1
		raw_idx = -1

		#read the each lines(rows) in csv files
		for line in rdr:
			raw_idx +=1
			#get the x, y position from now csv file
			x1 = float(line[0])
			y1 = float(line[1])

			#find nearest x, y 
			a = x2 - x1
			b = y2 - y1
			c = math.pow(a, 2) + math.pow(b, 2)
			if(c < min_dis):
				#save current nearest x,y position comapare with now mouse position from 2D Nav GOAl message
				result_x = x1
				result_y = y1
				min_dis = c
				#save current raw index
				check_idx = raw_idx
				#save current file's name (source node + destination node . csv )
				files_name = str(line_files[files_idx])
				#save the result position to use after 
				position_result = np.array([])
				position_result = np.append(position_result, np.array([line_files[files_idx], result_x, result_y,files_name]))
	f.close()  

	#find the nearest node from above name of saved files_name
	select_file = "/home/kanakim/catkin_ws/src/gui/map/" + files_name
	s = io.open(select_file, 'r', encoding='utf-8')
	rdr = csv.reader(s)
	csv_len = 0
	goal_node = '' #send_goal_node 

	#get the node's name 
	src_node = files_name[0:2] #first two 
	dst_node = files_name[2:4] #last two 

	#get the target csv file's length
	for cnt in rdr :
	    csv_len += 1

	#find the neareset node
	if ( check_idx < csv_len/2 ) : #if row index is smaller than half of csv files length, 
	    goal_node = src_node # mouse's postion is close to the source node
	elif (check_idx >= csv_len/2) : #else if row index is bigger than half of csv files length,
	    goal_node = dst_node #mouse's postion is close to the destination node
	else :
	    print("Error to find Goal Node")

	fin_position_x = result_x
	fin_position_y = result_y

	# goal point for distance calculation
	msg.twist.twist.linear.x = fin_position_x
	msg.twist.twist.linear.y = fin_position_y

	#Marker for Goal Point
	check = Marker()
	check.header.frame_id = "world"
	check.ns = "/check" #if user select goal
	check.type = check.SPHERE
	check.action = check.ADD
	check.lifetime = rospy.Duration(0)
	check.id = 1
	check.pose.position.x = fin_position_x
	check.pose.position.y = fin_position_y
	check.pose.position.z = 0.01
	check.pose.orientation.x = 0.0
	check.pose.orientation.y = 0.0
	check.pose.orientation.z = 0.0
	check.pose.orientation.w = 1.0
	check.scale.x = 0.7
	check.scale.y = 0.7
	check.scale.z = 0.002
	#rgb(241, 250, 140) yellow
	check.color.a = 1.0
	check.color.r = 0.95
	check.color.g = 0.98
	check.color.b = 0.55

	#Inform the goal point's information (x, y, yaw)
	send_goal = Point()
	send_goal.x = fin_position_x
	send_goal.y = fin_position_y
	send_goal.z = 0.0

	#check if the selected goal is correct
	print_goal = "-*-*-*-UPDATE GOAL POINT BY DRONE-*-*-*-\nGoal Node [{}] Raw Index [{}]\nGoal Point [{}, {}] Heading [{}]".format(goal_node, check_idx, fin_position_x, fin_position_y, 0.0)
	print(print_goal)
	goal_sent_successfully = True

	#publish all information about goal 
	#publish goal node number as stdstring
	pub_goal_node.publish(goal_node)
	#publish goal's information as unique message (float, float, float)
	pub_goal_pos.publish(send_goal)
	#publish goal's marker
	pub_check.publish(check)

def end_drone_obj_detection():
	global darknet_ros_proc
	global drone_object_detection_status
	psutil.Process(darknet_ros_proc.pid).send_signal(subprocess.signal.SIGINT)
	darknet_ros_proc.send_signal(subprocess.signal.SIGINT)
	drone_object_detection_status = False
	print("drone object detection ended")
	start_car_obj_detection()

def start_car_obj_detection() :
	global darknet_ros_proc
	global drone_object_detection_status
	print("car object detection started")
	if drone_object_detection_status == False :
		command ="roslaunch darknet_ros darknet_ros_car.launch"
		command = shlex.split(command)
		darknet_ros_proc = subprocess.Popen(command)
		

start_drone_object_detection()

pub_check = rospy.Publisher('/check', Marker, queue_size = 1, latch = True)
pub_goal_pos = rospy.Publisher('/goal_pos', Point,  queue_size = 1)
pub_goal_node = rospy.Publisher('/goal_node', String, queue_size=10)

rospy.init_node("visualizer", anonymous=True)
rospy.Subscriber('/darknet_ros/bounding_boxes_d', BoundingBoxes, objConditionCheck)
rospy.spin()


