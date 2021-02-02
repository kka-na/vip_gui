#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry


def p_test() :
	pub = rospy.Publisher('/pose_test', Odometry, queue_size = 1)
	rospy.init_node('pose_test', anonymous=True)
	r = rospy.Rate(30)
	send_point = Odometry()
	send_point.pose.pose.position.x = 126.654391
	send_point.pose.pose.position.y = 37.384020
	send_point.twist.twist.angular.z = 0.8862

	while not rospy.is_shutdown() :
		pub.publish(send_point)
		send_point.pose.pose.position.x += 0.000001
		send_point.pose.pose.position.y += 0.0000005
		print(send_point.pose.pose.position.x)
		r.sleep()


if __name__ == '__main__' :
	try :
		p_test()
	except rospy.ROSInterruptException: pass
