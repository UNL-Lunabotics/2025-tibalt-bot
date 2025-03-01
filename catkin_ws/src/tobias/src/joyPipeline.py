#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy



def joyCallback(data):
	joyData.axes = data.axes
	joyData.buttons = data.buttons

def start():
	#init node
	rospy.init_node('joyPipeline')
	
	#init vars
	rate = rospy.Rate(20)

	global joyData
	joyData = Joy()
	joyData.axes = [0,0,0,0,0,0]
	joyData.buttons = [0,0,0,0,0,0,0,0,0,0,0,0]

	#init pub/sub
	pubJoy = rospy.Publisher('joy_pipe', Joy, queue_size=10)
	rospy.Subscriber('joy',Joy,joyCallback)
	
	#main loop
	while not rospy.is_shutdown():
		pubJoy.publish(joyData)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	start()
