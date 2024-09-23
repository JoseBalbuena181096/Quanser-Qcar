## task_lane_following.py
# This example combines both the left csi and motor commands to 
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar 
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

from pal.products.qcar import QCar
from pal.utilities.math import *

import numpy as np 
import cv2

from lines import LaneDetect
from keys import KeyListener
from lidar import LidarProcessor
from camera import CameraProcessor
from control import ControlSystem

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
myCar = QCar()

lanes = LaneDetect()
lidar = LidarProcessor()
camera = CameraProcessor()
control = ControlSystem()
key_listener = KeyListener()
key_listener.start()
throttle_axis = 0 
steering_axis = 0

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

## Main Loop

try:
	while not key_listener.should_exit:
		frame = camera.take_frame()
		lines_frame = lanes.find_lines(frame)   
		try:        		     
			cv2.imshow('Camera', lines_frame )
		except:
			pass

		print(key_listener.last_key_pressed)
		
		if  key_listener.last_key_pressed == 'a':
			steering_axis = control.control_p(lanes.error)
			throttle_axis = 0.05
		if key_listener.last_key_pressed == 's':
			steering_axis = control.control_p(lanes.error)
			throttle_axis = 0   

		steering_axis =  control.saturate(steering_axis, 0.6, -0.6) 		    
		LEDs = np.array([0, 0, 0, 0, 1, 1, 0, 0])
		if lidar.detect_object():
			print("Object detected!")
			throttle_axis = 0
		myCar.read_write_std(throttle_axis,steering_axis ,LEDs)	
		cv2.waitKey(1)
	
except KeyboardInterrupt:
	print("User interrupted!")

finally:
	# Terminate camera and QCar
	myCar.terminate()
	lidar.end_lidar()
	camera.end_camera()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
