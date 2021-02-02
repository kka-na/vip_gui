

#!/usr/bin/env python
import rospy
import numpy as np
import time
import math
import tf
import csv
import io
import os, glob
import os.path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

count = 1
#mouse_count = 1

mapArray = MarkerArray()
mapArray.markers = []
msg = Odometry()


def world(file):
    global mapArray, count
    f = io.open(file, 'r', encoding='utf-8')
    rdr = csv.reader(f)
    print("==========================WORLD START=========================")

    for line in rdr:
        line.pop()
        line.pop()
        line.pop()
        
        print(line)

        marker = Marker()
        marker.header.frame_id = "world" # publish path in map frame
        marker.ns = "map" + str(count)
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.id = count
        marker.pose.position.x = float(line[0])
        marker.pose.position.y = float(line[1])
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        mapArray.markers.append(marker)
        count += 1


    f.close()

def mouseInterrupt(data):
    #global mouse_count
    position = (
        data.pose.position.x,
        data.pose.position.y)
    
    result_x = 0.0
    result_y = 0.0
    min_dis = 9999.9
    
    x1 = 0.0
    y1 = 0.0
    x2 = position[0]
    y2 = position[1]
    #print (type(position[0]))

    target_dir = "/home/kanakim/catkin_ws/src/gui/map/"
    files = os.listdir(target_dir)
    
    for number in files:
        #print(number)
        file = "/home/kanakim/catkin_ws/src/gui/map/" + number
        f = io.open(file, 'r', encoding='utf-8')
        rdr = csv.reader(f)

        for line in rdr:
            line.pop()
            line.pop()
            line.pop()
            #print (type(line[0]))
            x1 = float(line[0])
            y1 = float(line[1])

            a = x2 - x1
            b = y2 - y1
            c = math.pow(a, 2) + math.pow(b, 2)

            if(c < min_dis):
                result_x = x1
                result_y = y1
                min_dis = c 
        
        f.close()


    print(result_x)
    print(result_y)

    check = Marker()
    check.header.frame_id = "map" # publish path in map frame
    check.ns = "/check"
    check.type = check.SPHERE
    check.action = check.ADD
    check.lifetime = rospy.Duration(0)
    check.id = 1
    check.pose.position.x = result_x
    check.pose.position.y = result_y
    check.pose.position.z = 0
    check.pose.orientation.x = 0.0
    check.pose.orientation.y = 0.0
    check.pose.orientation.z = 0.0
    check.pose.orientation.w = 1.0
    check.scale.x = 0.04
    check.scale.y = 0.04
    check.scale.z = 0.04
    check.color.a = 1.0
    check.color.r = 1.0
    check.color.g = 0.0
    check.color.b = 0.0
    #mouse_count += 1
    #print(mouse_count)

    pub_mouse.publish(check)

world('/home/kanakim/catkin_ws/src/gui/map/0001.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0010.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0100.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0102.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0111.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0201.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0203.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0212.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0302.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0304.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0313.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0403.csv')
world('/home/kanakim/catkin_ws/src/gui/map/0414.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1000.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1011.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1020.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1101.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1110.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1112.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1121.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1202.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1211.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1213.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1222.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1303.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1312.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1312.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1314.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1323.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1404.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1413.csv')
world('/home/kanakim/catkin_ws/src/gui/map/1424.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2010.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2021.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2111.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2120.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2122.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2212.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2221.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2223.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2313.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2322.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2324.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2414.csv')
world('/home/kanakim/catkin_ws/src/gui/map/2423.csv')



pub_array = rospy.Publisher('/map', MarkerArray, queue_size = 100, latch = True)
pub_mouse = rospy.Publisher('/check', Marker, queue_size = 1, latch = True)
pub_start = rospy.Publisher('/start', Marker, queue_size = 1, latch = True)
pub = rospy.Publisher('/vehicle', Marker, queue_size = 1)
rospy.init_node("visualization",anonymous=True)
#rospy.Subscriber('/gps_data/fix',NavSatFix,GPS)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, mouseInterrupt)
#rospy.Subscriber('/imu/data', Imu, IMU)
pub_array.publish(mapArray)
rospy.spin()

'''
while not rospy.is_shutdown():
    print("/map" + str(count))
    
'''