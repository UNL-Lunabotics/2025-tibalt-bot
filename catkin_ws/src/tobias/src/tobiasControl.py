#! /usr/bin/env python3
from locale import strcoll
from smtplib import SMTPServerDisconnected
import rospy
import time
import rosgraph
import socket
import numpy as np
import cv2

import numpy as np
from PIL import Image


from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import CompressedImage

# excavation version: 0: both, 1: left, 2: right
excUsedSystem = 0
# auton code
# hopperPhase 1: vibe a bit to get the stuff loose and ready, 2: open hopper door, 3: close hopper door
hopperPhase = 0
# excPhase 1: fully extend industrial, 2: full retract bullet, 3: retract indust while driving trench
# 3: move drivetrain while driving trench, 4: extend indust while driving trench
excPhase = 0

excTimer = 0
hopperTimer = 0
bounce = 0
wasSwitchingTrench = 0
lastVibeStartTime = 0


def slidersCallback(arr):
	global sliderValues
	sliderValues = arr

def encCallback(arr):
	global Encoders  
	Encoders = arr.data

def joyCallback(data):
	global hopperPhase
	global excPhase
	global excUsedSystem
	global excTimer, hopperTimer
	global bounce
	global wasSwitchingTrench
	global sliderValues
	
	global Speeds
	global Encoders
	global camera_used

	(ExTrencherBulletLeft, ExTrencherBulletRight, ExTrencherIndLeft, ExTrencherIndRight) = (0,0,0,0)

	# if autonomy button is pressed, then sets autonomy on and sets the position to the top gate
	# if(data.buttons[1] and data.buttons[2]):
	# 	hopperPhase = 1
	if(data.buttons[1] and data.buttons[2]):
		excPhase = 1
		excTimer = time.time()
		lastVibeStartTime = time.time()
	if(data.buttons[1] and data.buttons[5]):
		# safeguard to end autonomy: press buttons 2 and 6
		excPhase = 0
	
	# data from joystick
	lrAxis = data.axes[0]
	fbAxis = data.axes[1]

	maxi = max(abs(fbAxis), abs(lrAxis))
	total = fbAxis + lrAxis
	diff = fbAxis - lrAxis

	# code to determine which trencher is moving
	if (data.buttons[4] == 1 and not wasSwitchingTrench):
		excUsedSystem += 1
		if(excUsedSystem > 2):
			excUsedSystem = 0

	wasSwitchingTrench = data.buttons[4]

	# drivetrain calculations
	DtRight = 64
	DtLeft = 64
	
	if(fbAxis > 0):
		if(lrAxis > 0):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
			DtLeft -= 63*diff
			DtRight += 63*maxi
		else:
			DtLeft -= 63*maxi
			DtRight += 63*total
	else:
		if(lrAxis > 0):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
			DtLeft += 63*maxi
			DtRight += 63*total
		else:
			DtLeft -= 63*diff
			DtRight += -63*maxi

	(ExTrencherIndRight, ExTrencherIndLeft, ExTrencherBulletRight, ExTrencherBulletLeft) = (64,64,64,64)
	(ExTrencherDriveLeft, ExTrencherDriveRight) = (64,64)
	(trencher_left_speed, trencher_right_speed, exc_auton_drive_speed, exc_auton_shake, exc_auton_drive_time, trencher_act_speed) = (sliderValues.data[0], sliderValues.data[1], sliderValues.data[2], sliderValues.data[3], sliderValues.data[4], sliderValues.data[5])

	# TODO: edit code based on how fast we want this to be able to go
	if(excUsedSystem == 0 or excUsedSystem == 1):
		ExTrencherDriveLeft = 64 + 1 * (data.buttons[0] * trencher_left_speed)
		# ExTrencherDriveLeft = sliderValues.data[0]	
		ExTrencherBulletLeft = 64 + 1 * (data.buttons[6] - data.buttons[7])*63
		ExTrencherIndLeft = 64 + 1 * (data.buttons[8] - data.buttons[9])*sliderValues.data[5]

	if(excUsedSystem == 0 or excUsedSystem == 2):
		ExTrencherDriveRight = 64 - 1 * (data.buttons[0] * trencher_right_speed)
		# ExTrencherDriveRight = sliderValues.data[1]
		ExTrencherBulletRight = 64 - 1 * (data.buttons[6] - data.buttons[7])*63
		ExTrencherIndRight = 64 - 1 * (data.buttons[8] - data.buttons[9])*sliderValues.data[5]

	HpOpen = 64 + (data.buttons[10] - data.buttons[11]) * 63  
	HpVibe = 64 + data.buttons[3] * ((data.axes[3] + 1) * 32)
	
	# TODO: add two slider values: time_to_actuate_hopper, time_to_actuate_exc_indust
	if(hopperTimer != 0):
		if hopperPhase == 1: 
			# run for 20 secs before next phase
			if((time.time() - excTimer) / 60 >= 20):
				excPhase = 2
			else:
				HpOpen = 64 + 63  
		if hopperPhase == 2:
			if((time.time() - excTimer) / 60 >= 10):
				excPhase = 3
			HpVibe = 64 + data.buttons[3] * ((data.axes[3] + 1) * 32)
		if hopperPhase == 3:
			if((time.time() - excTimer)/60 >= 20):
				excPhase = 0
			else:
				HpOpen = 64 - 63  

	# excavation autonomy:
	if(excPhase != 0):
		# run the vibe motor, 10 seconds on, ten seconds off 
		if (time.time() - lastVibeStartTime) / 60 > 20:
			lastVibeStartTime = time.time()
		elif (time.time() - lastVibeStartTime) / 60 < 10:
			HpVibe = 64 + data.buttons[3] * exc_auton_shake # uses exc auton shake value from slider
		else: 
			HpVibe = 64

		if excPhase == 1: 
			# run for 20 secs before next phase
			# TODO: add a slider for time to lower and time to raise
			if((time.time() - excTimer) / 60 >= 20):
				excPhase = 2
			else:
				if(excUsedSystem == 0 or excUsedSystem == 1):
					ExTrencherDriveLeft = 64 + trencher_left_speed
					ExTrencherIndLeft = 64 + 63
				if(excUsedSystem == 0 or excUsedSystem == 2):
					ExTrencherDriveRight = 64 - trencher_right_speed
					ExTrencherIndRight = 64 - 63
		# excPhase 2: has reached lowest stage, begin driving forwards/backwards while continuing digging
		elif(excPhase == 2):
			if((time.time() - excTimer) / 60 >= exc_auton_drive_time):
				excPhase = 2
				excTimer = time.time()
			else:
				# TODO: double check the pos/neg is working right for which way we want to drive
				DtLeft = 64 + exc_auton_drive_speed
				DtRight = 64 - exc_auton_drive_speed
				if(excUsedSystem == 0 or excUsedSystem == 1):
					ExTrencherDriveLeft = 64 + trencher_left_speed
				if(excUsedSystem == 0 or excUsedSystem == 2):
					ExTrencherDriveRight = 64 - trencher_right_speed
		# excPhase 3: begin pulling system back up (extend indust) while digging
		elif(excPhase == 3):
			# run for 25 secs before next phase
			if((time.time() - excTimer) / 60 >= 25):
				excPhase = 0
			else:
				if(excUsedSystem == 0 or excUsedSystem == 1):
					ExTrencherDriveLeft = 64 + trencher_left_speed
					ExTrencherIndLeft = 64 - 1 * 63
				if(excUsedSystem == 0 or excUsedSystem == 2):
					ExTrencherDriveRight = 64 - trencher_right_speed
					ExTrencherIndRight = 64 + 1 * 63
	
	Speeds.data = [int(DtLeft), int(DtRight), int(ExTrencherIndLeft), int(ExTrencherBulletLeft), int(ExTrencherIndRight), int(ExTrencherBulletRight),int(ExTrencherDriveLeft), int(ExTrencherDriveRight), int(HpVibe), int(HpOpen)]

def start():
	#init node
	rospy.init_node('Tobias')
	
	#init vars
	rate = rospy.Rate(10)

	global Speeds  
	Speeds = Int16MultiArray()

	global CurrentImage
	CurrentImage = CompressedImage()

	global Encoders
	Encoders = Int16MultiArray()

	global sliderValues
	sliderValues = Int16MultiArray()

	#init pub/sub
	pubJoy = rospy.Publisher('TMP', Int16MultiArray, queue_size=10) # publishes the speeds

	rospy.Subscriber('slider_values', Int16MultiArray, slidersCallback)
	rospy.Subscriber('joy_pipe', Joy, joyCallback)
	# rospy.Subscriber('enc_pipe', Int16MultiArray, encCallback)
	sliderValues.data = [64, 64]
	rospy.Subscriber('enc_pipe', Int16MultiArray, encCallback)
	sliderValues.data = [64, 64, 64, 64, 64, 64, 1]

	#main loop
	while not rospy.is_shutdown():
		socket.setdefaulttimeout(1)
		if rosgraph.is_master_online():
			socket.setdefaulttimeout(None)
			pubJoy.publish(Speeds)
		else:
			socket.setdefaulttimeout(None)
			Speeds.data = [64, 64, 64, 64, 64, 64, 64, 64, 64, 64]
			pubJoy.publish(Speeds)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	start()
