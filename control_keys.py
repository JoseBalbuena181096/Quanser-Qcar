## task_lane_following.py
# This example combines both the left csi and motor commands to 
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar 
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

from pal.products.qcar import QCar
from pal.utilities.math import *


from keys import KeyListener
import numpy as np


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
myCar = QCar()

key_listener = KeyListener()
key_listener.start()
throttle_axis = 0 
steering_axis = 0

try:
	while not key_listener.should_exit:
		print(key_listener.last_key_pressed)
		
		if key_listener.last_key_pressed == 'A':
			print('FRONT')
			throttle_axis = 0.05
		elif key_listener.last_key_pressed == 'B':
			print('BACK')
			throttle_axis = -0.05
		elif key_listener.last_key_pressed == 'C':
			print('RIGHT')
			steering_axis = 0.35
		elif key_listener.last_key_pressed == 'D':
			print('LEFT')
			steering_axis = -0.35
		elif key_listener.last_key_pressed == 's':
			print('STOP')
			steering_axis = 0
			throttle_axis = 0
			
		LEDs = np.array([0, 0, 0, 0, 1, 1, 0, 0])
		myCar.read_write_std(throttle_axis,steering_axis ,LEDs)		
	
except KeyboardInterrupt:
	print("User interrupted!")

finally:
	# Terminate camera and QCar
	myCar.terminate()

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
