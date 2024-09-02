#!/usr/bin/python3
import rospy
from std_msgs.msg import UInt64
from std_msgs.msg import Float32

k = 0
result = 0
q = 0.15
def callback_pal(msg):
	k = msg.data
	global result 
	result = k/q

	
def sub_fun():
	rospy.Subscriber("pal",UInt64, callback_pal)
	pub  = rospy.Publisher('/kthfs/result', Float32, queue_size = 5)
	rospy.init_node('result_node', anonymous = True)
	
	rospy.loginfo("Subscribing to pal_node and publishing result")
	rate = rospy.Rate(20);
	while not rospy.is_shutdown():
		pub.publish(result)
		rate.sleep()
		
if __name__ == '__main__':
	try:
		sub_fun()
	except rospy.ROSInterruptException:
		pass
		
	
