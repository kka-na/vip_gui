"""
#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

send_point = Odometry()
send_point.pose.pose.position.x = 126.654310
send_point.pose.pose.position.y = 37.383784
send_point.twist.twist.angular.z = 0.8862

def k_callback(data) :
	global send_point
	pub.publish(send_point) 
	print(send_point.pose.pose.position.y,send_point.pose.pose.position.x )
	if data.linear.x == 0.5 :
		send_point.pose.pose.position.y += 0.000001
	elif data.linear.x == -0.5 :
		send_point.pose.pose.position.y -= 0.000001
	elif data.angular.z == 1.0 :
		send_point.pose.pose.position.x += 0.000001
	elif data.angular.z == -1.0 :
		send_point.pose.pose.position.x -= 0.000001

	
pub = rospy.Publisher('/pose_test', Odometry, queue_size = 1)
rospy.init_node('pose_test', anonymous=True)
rospy.Subscriber('/cmd_vel', Twist, k_callback)	
rospy.spin()
"""

#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry


def p_test() :
	pub = rospy.Publisher('/pose', Odometry, queue_size = 1)
	rospy.init_node('pose', anonymous=True)
	r = rospy.Rate(10)
	send_point = Odometry()
	send_point.pose.pose.position.x = 126.654310
	send_point.pose.pose.position.y = 37.383784
	send_point.twist.twist.angular.z = 0.8862

	while not rospy.is_shutdown() :
		pub.publish(send_point)
		send_point.pose.pose.position.x -= 0.00000001
		send_point.pose.pose.position.y += 0.000001
		send_point.twist.twist.angular.z -= 0.0001
		print(send_point.pose.pose.position.x)
		r.sleep()


if __name__ == '__main__' :
	try :
		p_test()
	except rospy.ROSInterruptException: pass