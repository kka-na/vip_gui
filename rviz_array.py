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

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
#from obstacle_detector.msg import Obstacles
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from geometry_msgs.msg import Point


count = 1
gps_count = 1

#for map display
worldArray = MarkerArray()
worldArray.markers = []
#for publish car's current information 
msg = Odometry()

"""
LPP_Array = MarkerArray()
LPP_Array = []

GPP_Array = MarkerArray()
GPP_Array = []

obstacle_Array = MarkerArray()
obstacle_Array.markers = []
"""


def world(file):
    global worldArray, count
    f = io.open(file, 'r', encoding='utf-8')
    rdr = csv.reader(f)

    for line in rdr :
        marker = Marker()
        #set the frame ID and time stamp
        marker.header.frame_id = "world" #publish map named world frame
        #set the namespace and id for current marker, this serves to create a unique ID 
        marker.ns = "map" + str(count)
        marker.id = count
        #set the marker type 
        marker.type = marker.SPHERE
        #set the marker action (OPTIONS : ADD, DELETE, NEW )
        marker.action = marker.ADD
        #set lifetime ( duration )
        marker.lifetime = rospy.Duration(0)

        #set the pose of the marker ( 6DOF pose relative to frame/time )
        marker.pose.position.x = float(line[0])
        marker.pose.position.y = float(line[1])
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        #set the scale of the marker ( 1*1*1 means 1m on a side )
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.001
        #set the color of the marker ( rgba / 255 ), rgb(255, 121, 198) pink
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.47
        marker.color.b = 0.78
        #append the markers to markerarray
        worldArray.markers.append(marker)
        count += 1

    f.close()

def findgoal(data): 
    position = (
        data.pose.position.x,
        data.pose.position.y)

    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    #initialize
    result_x = 0.0
    result_y = 0.0
    min_dis = 9999.9
    x1 = 0.0
    y1 = 0.0
    x2 = position[0] #x position selected with mouse ( rviz's 2D NAV GOALS )
    y2 = position[1] #y position selected with mouse ( rviz's 2D NAV GOALS )

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

    #Inform the goal point's information (x, y , yaw)
    send_goal = Point()
    send_goal.x = fin_position_x
    send_goal.y = fin_position_y
    send_goal.z = euler[2]

    #check if the selected goal is correct
    print_goal = "-*-*-*-USER UPDATE GOAL POINT-*-*-*-\nGoal Node [{}] Raw Index [{}]\nGoal Point [{}, {}] Heading [{}]".format(goal_node, check_idx, fin_position_x, fin_position_y, euler[2])
    print(print_goal)
    
    #publish all information about goal 
    #publish goal node number as stdstring
    pub_goal_node.publish(goal_node)
    #publish goal's information as unique message (float, float, float)
    pub_goal_pos.publish(send_goal)
    #publish goal's marker
    pub_mouse.publish(check)



#------- this is about LiDAR -----------#
"""
def obstacle(data):
    ob_num = Odometry() # parameter messege (obstacle's count)

    # CYLINDER
    for ob_count in range (len(data.circles)):
        obstacle_Marker = Marker()
        obstacle_Marker.header.frame_id = "world" # publish path in map frame
        obstacle_Marker.header.stamp = rospy.Time.now()
        obstacle_Marker.ns = "circles"
        obstacle_Marker.type = obstacle_Marker.CYLINDER
        obstacle_Marker.action = obstacle_Marker.ADD
        obstacle_Marker.lifetime = rospy.Duration(0.1)
        obstacle_Marker.id = ob_count
        obstacle_Marker.pose.position.x = data.circles[ob_count].center.x + msg.pose.pose.position.x
        obstacle_Marker.pose.position.y = data.circles[ob_count].center.y + msg.pose.pose.position.y
        obstacle_Marker.pose.position.z = 0
        obstacle_Marker.pose.orientation.x = 0.0
        obstacle_Marker.pose.orientation.y = 0.0
        obstacle_Marker.pose.orientation.z = 0.0
        obstacle_Marker.pose.orientation.w = 1.0
        obstacle_Marker.scale.x = data.circles[ob_count].radius*1.4
        obstacle_Marker.scale.y = data.circles[ob_count].radius*1.4
        obstacle_Marker.scale.z = 0.5
        obstacle_Marker.color.a = 1.0
        obstacle_Marker.color.r = 0.0
        obstacle_Marker.color.g = 1.0
        obstacle_Marker.color.b = 0.0

        # Less than 0.01 radius cylinders remove 
        if(data.circles[ob_count].radius>0.01):
            obstacle_Array.markers.append(obstacle_Marker)


    # LINE_LIST
    for ob_count in range (len(data.segments)):
        obstacle_Marker = Marker()
        obstacle_Marker.header.frame_id = "world" # publish path in map frame
        obstacle_Marker.ns = "segments"
        obstacle_Marker.type = obstacle_Marker.LINE_LIST
        obstacle_Marker.action = obstacle_Marker.ADD
        obstacle_Marker.lifetime = rospy.Duration(0.1)
        obstacle_Marker.id = ob_count
        obstacle_Marker.pose.position.x = 0.0
        obstacle_Marker.pose.position.y = 0.0
        obstacle_Marker.pose.position.z = 0.0
        obstacle_Marker.pose.orientation.x = 0.0
        obstacle_Marker.pose.orientation.y = 0.0
        obstacle_Marker.pose.orientation.z = 0.0
        obstacle_Marker.pose.orientation.w = 1.0
        obstacle_Marker.scale.x = 0.04

        obstacle_Marker.color.a = 1.0
        obstacle_Marker.color.r = 0.0
        obstacle_Marker.color.g = 1.0
        obstacle_Marker.color.b = 0.0

        data.segments[ob_count].first_point.x += msg.pose.pose.position.x
        data.segments[ob_count].first_point.y += msg.pose.pose.position.y
        data.segments[ob_count].last_point.x += msg.pose.pose.position.x
        data.segments[ob_count].last_point.y += msg.pose.pose.position.y
        
        obstacle_Marker.points.append(data.segments[ob_count].first_point)
        obstacle_Marker.points.append(data.segments[ob_count].last_point)
        obstacle_Array.markers.append(obstacle_Marker)
    
    # to rviz transmit
    pub_obstacle.publish(obstacle_Array)
    
    # to QT transmit (obstacle's count)
    ob_num.pose.pose.position.x = len(obstacle_Array.markers)
    pub_obstacle_number.publish(ob_num)

    # obstacles clean
    for ob_count in range (len(obstacle_Array.markers)):
        obstacle_Array.markers.pop()

def LPP(data):
    for LPP_count in range (len(data.points)):
        LPP = Marker()
        LPP.header.frame_id = "world" #+ str(ob_count) # publish path in map frame
        LPP.ns = "LPP" + str(LPP_count)
        LPP.type = LPP.CUBE
        LPP.action = LPP.ADD
        LPP.lifetime = rospy.Duration(0)
        LPP.id = LPP_count
        LPP.pose.position.x = data.points[LPP_count].x + msg.pose.pose.position.x
        LPP.pose.position.y = data.points[LPP_count].y + msg.pose.pose.position.y
        LPP.pose.position.z = 0
        LPP.pose.orientation.x = 0.0
        LPP.pose.orientation.y = 0.0
        LPP.pose.orientation.z = 0.0
        LPP.pose.orientation.w = 1.0
        LPP.scale.x = 0.1
        LPP.scale.y = 0.1
        LPP.scale.z = 0.0015
        LPP.color.a = 1.0
        LPP.color.r = 1.0
        LPP.color.g = 1.0
        LPP.color.b = 0.0
    
        LPP_Array.markers.append(LPP)


def GPP(data):
    for GPP_count in range (len(data.points)):
        GPP = Marker()
        GPP.header.frame_id = "world" #+ str(ob_count) # publish path in map frame
        GPP.ns = "GPP" + str(GPP_count)
        GPP.type = GPP.CUBE
        GPP.action = GPP.ADD
        GPP.lifetime = rospy.Duration(0)
        GPP.id = GPP_count
        GPP.pose.position.x = data.points[GPP_count].x + msg.pose.pose.position.x
        GPP.pose.position.y = data.points[GPP_count].y + msg.pose.pose.position.y
        GPP.pose.position.z = 0
        GPP.pose.orientation.x = 0.0
        GPP.pose.orientation.y = 0.0
        GPP.pose.orientation.z = 0.0
        GPP.pose.orientation.w = 1.0
        GPP.scale.x = 0.1
        GPP.scale.y = 0.1
        GPP.scale.z = 0.0015
        GPP.color.a = 1.0
        GPP.color.r = 0.0
        GPP.color.g = 1.0
        GPP.color.b = 0.0
    
        GPP_Array.markers.append(GPP)
"""

#if get the gps, imu information from CAR , subscribe call back function
def GPSIMU(data):
    global gps_count 
    
    # base point offset parameter
    base_lat = 37.383784    
    base_lon = 126.654310
    base_alt = 30.3
    
    #get the lat, lng from gps 
    lat = float(data.pose.pose.position.y)
    lng = float(data.pose.pose.position.x)
    alt = 34.5

    # latitude, longitude  -->  x, y use pymap3d
    x, y, u = pymap3d.geodetic2enu(lat, lng, alt, base_lat, base_lon, base_alt)

    #converted lat->x, lng->y for display now car's position
    #vehicle's position information for publish
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    #print( msg.pose.pose.position.x,  msg.pose.pose.position.y)
    # Distance calculation
    distance = abs(math.hypot(msg.twist.twist.linear.x-msg.pose.pose.position.x,
        msg.twist.twist.linear.y-msg.pose.pose.position.y))
    msg.twist.twist.linear.z = distance
    

    # 1 second after, start point setting
    if (gps_count == 2):
        #initializing
        result_x = 0.0
        result_y = 0.0
        min_dis = 999999.9
        gps_count += 1
        x1 = 0.0
        y1 = 0.0
        x2 = x #converted x from GPS current latitude
        y2 = y #converted y from GPS current longitude
        point_yaw = msg.pose.pose.position.z #???

        target_dir = "/home/kanakim/catkin_ws/src/gui/map"
        files = os.listdir(target_dir)

        files_idx = -1
        check_idx = 0

        for number in files:
            file = "/home/kanakim/catkin_ws/src/gui/map/" + number
            f = io.open(file, 'r', encoding='utf-8')
            rdr = csv.reader(f)

            #line_files.append(number)
            files_idx += 1
            raw_idx = -1
            
            for line in rdr:
                raw_idx +=1

                x1 = float(line[0])
                y1 = float(line[1])
            
                a = x2 - x1
                b = y2 - y1
                c = math.pow(a, 2) + math.pow(b, 2)

                if(c < min_dis):
                    result_x = x1
                    result_y = y1
                    min_dis = c
                    check_idx = raw_idx
                    
        f.close()  

        fin_position_x = result_x
        fin_position_y = result_y
        
        #Make starting position maker
        start = Marker()
        start.header.frame_id = "world" 
        start.ns = "/start"
        start.type = start.CUBE
        start.action = start.ADD
        start.lifetime = rospy.Duration(0)
        start.id = 1
        start.pose.position.x = fin_position_x
        start.pose.position.y = fin_position_y
        start.pose.position.z = 0
        start.pose.orientation.x = 0.0
        start.pose.orientation.y = 0.0
        start.pose.orientation.z = 0.0
        start.pose.orientation.w = 1.0
        start.scale.x = 0.7
        start.scale.y = 0.7
        start.scale.z = 0.005
        start.color.a = 1.0
        #rgb(255, 184, 108) orange
        start.color.r = 1.0
        start.color.g = 0.72
        start.color.b = 0.42
        pub_start.publish(start)

    #if start position was set, display now car position 
    elif (gps_count > 2) :
        # vehicle display
        vehicle = Marker()
        vehicle.header.frame_id = "world" 
        vehicle.ns = "/vehicle"
        vehicle.type = vehicle.CUBE
        vehicle.action = vehicle.ADD
        vehicle.lifetime = rospy.Duration(0)
        vehicle.id = 1
        #converted x, y position from gps lat, lng
        vehicle.pose.position.x = msg.pose.pose.position.x

        vehicle.pose.position.y = msg.pose.pose.position.y
        vehicle.pose.position.z = 0
        vehicle.pose.orientation.x = 0
        vehicle.pose.orientation.y = 0
        #heading point 
        vehicle.pose.orientation.z =0.0 #this is yaw data
        vehicle.pose.orientation.w = 1.0 # ???\

        #vehicle.pose.orientation.z = data.pose.pose.orientation.z #this is yaw data
        #vehicle.pose.orientation.w = data.pose.pose.orientation.w # ????
        
        vehicle.scale.x = 0.4 
        vehicle.scale.y = 0.8
        vehicle.scale.z = 0.3
        vehicle.color.a = 1.0
        vehicle.color.r = 0.31
        vehicle.color.g = 0.98
        vehicle.color.b = 0.48
        #publish now vehicle's position
        pub.publish(vehicle)

    #wait about 2 seconds 
    elif (gps_count < 3) :
        gps_count += 1
    else : 
        pass

    #vehicle's yaw information for publish
    msg.pose.pose.position.z = data.pose.pose.position.z*3.141592/180 #this is yaw data
    
    #pubish now vehicle's position information
    pub_distance.publish(msg)


# ==========================================Map Data loading=============================================

def make_world() :
    path = '/home/kanakim/catkin_ws/src/gui/map/*'
    file_list = glob.glob(path)
    for file in file_list :
        world(file)

make_world()

#================================Publishe, Subscribe, and Excution================================9**h*
pub_array = rospy.Publisher('/map', MarkerArray, queue_size = 100, latch = True)
pub_mouse = rospy.Publisher('/check', Marker, queue_size = 1, latch = True)
pub_goal_pos = rospy.Publisher('/goal_pos', Point,  queue_size = 1)
pub_goal_node = rospy.Publisher('/goal_node', String, queue_size=10)

pub_start = rospy.Publisher('/start', Marker, queue_size = 1, latch = True)
#pub_obstacle_number = rospy.Publisher('/obstacle_number', Odometry, queue_size = 1)
#pub_obstacle = rospy.Publisher('/obstacle_rviz', MarkerArray, queue_size = 1)
pub_distance = rospy.Publisher('/distance', Odometry, queue_size = 1)
#pub_GPP = rospy.Publisher('/map_gpp', MarkerArray, queue_size = 1, latch = True)
#pub_LPP = rospy.Publisher('/map_lpp', MarkerArray, queue_size = 1, latch = True)
#pub_text = rospy.Publisher('/text', Marker, queue_size = 100, latch = True)
pub = rospy.Publisher('/vehicle', Marker, queue_size = 1)

rospy.init_node("visualization",anonymous=True)
rospy.Subscriber('/pose_test', Odometry, GPSIMU)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, findgoal)
#rospy.Subscriber('/obstacles', Obstacles, obstacle)
#rospy.Subscriber('/GPP', PointCloud, GPP)
pub_array.publish(worldArray)
rospy.spin()