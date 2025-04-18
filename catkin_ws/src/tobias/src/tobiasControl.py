#!/usr/bin/env python2
from locale import strcoll
from smtplib import SMTPServerDisconnected
import rospy
from datetime import datetime
import rosgraph
import socket
from detectBeacon import percentGreen
import numpy as np
import cv2
import time
# import image_transport
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Joy, Image
# from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray, Float32 
# excavation version: 0: both, 1: left, 2: right
excUsedSystem = 0
# excPhase 1: fully extend industrial, 2: full retract bullet, 3: retract indust while driving trench
# 3: move drivetrain while driving trench, 4: extend indust while driving trench
excPhase = 0
hopperPhase = 0

excTimer = 0
hopperTimer = 0

wasSwitchingTrench = 0
lastVibeStartTime = 0


def slidersCallback(arr):
	global sliderValues
	sliderValues = arr

def currentCallback(arr):
	global currentValues
	currentValues = arr		

def imageCallback(ros_data):
	global hopperImage
	try:
		hopperImage = bridge.imgmsg_to_cv2(ros_data, "bgr8")
	except CvBridgeError as e:
		print("error")

def joyCallback(data):
	global excPhase
	global excUsedSystem
	global excTimer
	global hopperPhase
	global hopperTimer
	global wasSwitchingTrench
	global sliderValues
	global currentValues
	global lastVibeStartTime
	global hopperImage
	
	# data from joystick
	lrAxis = data.axes[0]
	fbAxis = data.axes[1]

	# drivetrain calculations
	forward = int(fbAxis*63)
	turn = int(lrAxis*63)

	DtLeft = 64 + forward + turn
	DtRight = 64 + forward - turn

	# (trencher_left_speed, trencher_right_speed, exc_auton_drive_speed, exc_auton_shake, exc_auton_drive_time, trencher_act_speed) = (sliderValues.data[0], sliderValues.data[1], sliderValues.data[2], sliderValues.data[3], sliderValues.data[4], sliderValues.data[5])

	hp_hatch = 64 + (data.buttons[10] - data.buttons[11]) * 63  
	hp_vibe = 64 + data.buttons[3] * ((data.axes[3] + 1) * 32)

	Speeds.data =[int(ex_spin), int(ex_actuator), int(dt_left), int(dt_right), int(hp_hatch), int(hp_actuator), int(hp_vibe)]

def start():
	#init node
	rospy.init_node('Tobias')
	
	#init vars
	rate = rospy.Rate(10)

	global bridge
	bridge = CvBridge()

	global Green 
	Green = Float32()
	Green.data = 0

	global Speeds  
	Speeds = Int16MultiArray()
	#[ex_spin, ex_actuator, dt_left, dt_right, hp_hatch, hp_actuator, hp_vibe]
	Speeds.data = [64, 64, 64, 64, 64, 64, 64]

	global currentValues
	currentValues = Int16MultiArray()
	currentValues.data = [0,0]

	global sliderValues
	sliderValues = Int16MultiArray()

	#init pub/sub
	pubJoy = rospy.Publisher('TMP', Int16MultiArray, queue_size=10) # publishes the speeds
	pubGreen = rospy.Publisher('green', Float32, queue_size=1) # publishes the speeds

	rospy.Subscriber('slider_values', Int16MultiArray, slidersCallback)
	rospy.Subscriber('joy_pipe', Joy, joyCallback)
	rospy.Subscriber('current_pipe', Int16MultiArray, currentCallback)

	# TODO: figure out which is 1 vs 2
	rospy.Subscriber("usb_cam1/image_raw", Image, imageCallback)

	sliderValues.data = [63, 63, 15, 15, 15, 63, 1]


	#main loop
	while not rospy.is_shutdown():
		socket.setdefaulttimeout(1)
		if rosgraph.is_master_online():
			socket.setdefaulttimeout(None)
			pubJoy.publish(Speeds)
			pubGreen.publish(Green)
		else:
			socket.setdefaulttimeout(None)
			Speeds.data = [64, 64, 64, 64, 64, 64, 64]
			pubJoy.publish(Speeds)
			Green.data = 0
			pubGreen.publish(Green)

		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	start()