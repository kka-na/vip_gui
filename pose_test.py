#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

def p_test() :
	pub = rospy.Publisher('/pose_test', Point, queue_size = 1)
	rospy.init_node('pose_test', anonymous=True)
	r = rospy.Rate(10)
	send_point = Point()
	send_point.x = -1.57177733875
	send_point.y = 5.47445167845
	send_point.z = 0.0

	while not rospy.is_shutdown() :
		pub.publish(send_point)
		r.sleep()



if __name__ == '__main__' :
	try :
		p_test()
	except rospy.ROSInterruptException: pass
