#!/usr/bin/env python
# -*-coding: utf-8-*
import serial
import rospy
import roslib
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
import math
import struct

#ser = serial.Serial('/dev/ttyUSB0', 115200)

def nothing(x):
    pass

def roi(frame, vertices):
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask,vertices,255)
    masked = cv2.bitwise_and(frame,mask)
    return masked

def initializeTracbars(init_val):
    cv2.namedWindow("BEV_values")
    cv2.resizeWindow("BEV_values",360,240)
    cv2.createTrackbar("Width Top", "BEV_values",init_val[0],100,nothing)
    cv2.createTrackbar("Height Top", "BEV_values", init_val[1], 200, nothing)
    cv2.createTrackbar("Width Bottom", "BEV_values", init_val[2], 100, nothing)
    cv2.createTrackbar("Height Bottom", "BEV_values", init_val[3], 200, nothing)

def valTracbars():
    widthTop = cv2.getTrackbarPos("Width Top", "BEV_values")
    heightTop = cv2.getTrackbarPos("Height Top", "BEV_values")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "BEV_values")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "BEV_values")
    heightBottom = 117
    src = np.float32([(widthTop / 100, heightTop / 100), (1 - (widthTop / 100), heightTop / 100),
                      (widthBottom / 100, heightBottom / 100), (1 - (widthBottom / 100), heightBottom / 100)])
    return src

def perspective_warp(img,
                     dst_size=(1920, 1080),
                     src=np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)]),
                     dst=np.float32([(0,0), (1, 0), (0,1), (1,1)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def drawPoints(img, src):
    img_size = np.float32([(img.shape[1], img.shape[0])])
    # src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    src = src * img_size
    for x in range(0, 4):
        cv2.circle(img, (int(src[x][0]), int(src[x][1])), 10, (0, 0, 255), cv2.FILLED)
    return img

def color_filter(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    ycrcb = cv2.cvtColor(img,cv2.COLOR_BGR2YCrCb)
    lowThreshold_y = np.array([0,50,200])
    higherThreshold_y = np.array([25,120,255])
    lowThreshold_w = np.array([200,110,105])
    higherThreshold_w = np.array([255, 140, 130])
    lowThreshold_r = np.array([0,0,200])
    higherThreshold_r = np.array([100,200,255])
    masked_y = cv2.inRange(hsv,lowThreshold_y,higherThreshold_y)
    masked_w = cv2.inRange(ycrcb,lowThreshold_w,higherThreshold_w)
    masked_r = cv2.inRange(ycrcb,lowThreshold_r,higherThreshold_r)
    # cv2.imshow('y',mask_y)
    # cv2.imshow('w',mask_w)
    # cv2.waitKey(0)

    return masked_y, masked_w, masked_r

def edgeDetection(img):
    img=cv2.resize(img,(600,400),None)

    closed_img = cv2.erode(img, (5, 5), iterations=5)
    closed_img = cv2.dilate(closed_img, (3, 3), iterations=10)
    # cv2.imshow('closing', closed_img)
    edge_img = cv2.bitwise_or(cv2.Canny(closed_img,180,200), closed_img)
    return edge_img

def serWrite(speed, steering, cnt):

    break_val = 0x01
    #print('speed : ', speed);
    #print("ser_write", speed, steering, cnt)
    # steering 값 2000 넘길 시 2000으로 설정
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000
    # 기어 기본값 0: 전진, 1:후진
    #print("speed ", speed, "steering ", steering, "cnt : ", cnt)
    result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
   # print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
      #result[9], result[10], result[11], result[12], result[13] )
    ser.write(result)
    
    

# img = cv2.imread("test3.jpg")
# a,b=color_filter(img)
# edgeDetection(b)
#init_val = [25,30,7,70]   #wT,hT,wB,hB
init_val = [15,26,0,36]   #wT,hT,wB,hB
# video = cv2.VideoCapture("test_1.mp4") # port number check

video = cv2.VideoCapture(0) # port number check


initializeTracbars(init_val)

hough_val =0.35
signal = 0

pub_img = rospy.Publisher('/lane_detection/image_raw/compressed', CompressedImage,  queue_size = 1)
rospy.init_node("publisher",anonymous=True)
image_msg  = CompressedImage()

while True:
    BEV_values= valTracbars()
    _,orign_frame = video.read()
    frame=cv2.resize(orign_frame,(600,400),None)
    points_img = frame.copy()
    height,width = frame.shape[:2]
    points_img = drawPoints(points_img,BEV_values)
    frame= perspective_warp(frame,dst_size=(width,height),src = BEV_values)
    result_img = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    masked_y, masked_w, masked_r = color_filter(frame)
    edge_y = edgeDetection(masked_y)
    edge_w = edgeDetection(masked_w)
    edge_r = edgeDetection(masked_r)

    roi_height, roi_width = edge_w.shape[:2]
    sum_angle = 0
    cnt_angle = 0
    steering = 0
    w_lines = cv2.HoughLinesP(edge_w, 1, np.pi / 180, 50, maxLineGap=10)

    if w_lines is not None:
        for line in w_lines:
            x1,y1,x2,y2 = line[0]
            if(abs(y1-y2)>roi_height*hough_val and abs(x1-x2)<roi_width*0.5):
                cv2.line(result_img, (x1, y1), (x2, y2), (255, 255, 255), 5)
                tmp_angle = math.degrees(math.atan2(x2 - x1, y2 - y1))
                if (90 <= tmp_angle and tmp_angle <= 180): tmp_angle -= 180
                sum_angle += tmp_angle * (-1)
                cnt_angle += 1
            elif (abs(y1 - y2) < roi_height * 0.05 and abs(x1 - x2) > roi_width * 0.25):
                cv2.line(frame, (x1, y1), (x2, y2), (25, 25, 255), 5)
                signal = 1;

    y_lines = cv2.HoughLinesP(edge_y, 1, np.pi / 180, 50, maxLineGap=30)
    if y_lines is not None:
        for line in y_lines:
            x1, y1, x2, y2 = line[0]
            if (abs(y1 - y2) > roi_height * 0.1 and abs(x1 - x2) < roi_width * 0.4):
                cv2.line(result_img, (x1, y1), (x2, y2), (51, 204, 255), 5)
                tmp_angle = math.degrees(math.atan2(x2 - x1, y2 - y1))
                if (90 <= tmp_angle and tmp_angle <= 180): tmp_angle -= 180
                sum_angle += tmp_angle * (-1)
                cnt_angle += 1

    if (cnt_angle != 0):
        # print(round(sum_angle / cnt_angle, 2))
        final_ang = round(sum_angle / cnt_angle, 2)
        steering = final_ang
        if (abs(final_ang) > 15):
            hp_tf_val = 0.1
        else:
            hp_tf_val = 0.35

        #print(hp_tf_val)
        msg = str(round(sum_angle / cnt_angle, 2)) + "'"
        cv2.putText(frame, msg, (30, 30), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255), 2)

   # cv2.imshow('origin',points_img)
    #cv2.imshow('result', result_img)
    #cv2.imshow('yellow',edge_y)
    #cv2.imshow('white',edge_w)
    #cv2.imshow('red',edge_r)
    #cv2.imshow('masked_w',masked_w)
    #cv2.imshow('masked_y',masked_y)
    #cv2.imshow('masked_r',masked_r)

    speed_lim = 3
    if(abs(steering>30)):
        if(steering>0):
            steering = 30
        else:
            steering = -30

#############################################################serial 
    # print("===== steering :", steering)
    # result = ser.readline() # erp -> pc
    # print(result)   
    # print("len", len(result)) 
    # if len(result) > 17:
    #     print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
    #     result[9], result[10], result[11], result[12], result[13], result[14], result[15], result[16], result[17])
    #     cnt = result[15]
    #     print(cnt)
    #     serWrite(int(speed_lim*10), int(steering*71), cnt)
    # # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
    # elif len(result) == 16:
    #     print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
    #     result[9], result[10], result[11], result[12], result[13], result[14], result[15])
    #     add_result = ser.readline() # erp -> pc
    #     print(cnt)        
    #     serWrite(int(speed_lim*10), int(steering*71), 10)
    #     # serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
#############################################################

    image_msg.header.stamp = rospy.Time.now()
    image_msg.format =  "jpeg"
    encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),20]
    image_msg.data = np.array(cv2.imencode('.jpg', result_img, encode_param)[1]).tostring()
    pub_img.publish(image_msg)

    key = cv2.waitKey(1)
    if key == 27:
        break

    # p.pose.pose.orientation.y = signal
    # pub.publish(p)

    
video.release()
cv2.destroyAllWindows()