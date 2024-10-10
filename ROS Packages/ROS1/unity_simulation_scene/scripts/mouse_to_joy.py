#!/usr/bin/env python

# Siemens AG, 2018
# Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from Xlib import display
from Xlib.ext import randr
    
def mouseToJoy():
	# initialize node
	rospy.init_node('mouseToJoy', anonymous = True)

	#### Setup MouseToJoy Publisher 
   	mouseToJoyPublisher = rospy.Publisher("joy", Joy, queue_size = 5)
	rate = rospy.Rate(10) # 10hz
	msg = Joy()
	
	while not rospy.is_shutdown():
		#### Initialize joy msg every loop
		msg.axes = []
		msg.buttons = []
		pos_x = 0.0
		pos_y = 0.0
		
		#### Get Display Dependent Parameters
		d = display.Display()
		screen = d.screen()
		window = screen.root.create_window(0, 0, 1, 1, 1, screen.root_depth)
		res = randr.get_screen_resources(window)

		resolution_x = res.modes[0].width	# i.e. 1920
		resolution_y = res.modes[0].height	# i.e. 1080
		middlePoint_x = resolution_x / 2.0
		middlePoint_y = resolution_y / 2.0 
		
		#### Start Getting Postion of Mouse
		MouseData = display.Display().screen().root.query_pointer()._data
		pos_x = MouseData["root_x"]
		pos_y = MouseData["root_y"]

		#### Start Mapping from Mouse Position to Joy
		vel_linear  = (pos_y - middlePoint_y) / resolution_y * (2)
		vel_angular = (pos_x - middlePoint_x) / resolution_x * (2)	
	
		msg.axes.append(vel_linear)
		msg.axes.append(vel_angular)		


		#### Publish msg
		#rospy.loginfo([pos_x, pos_y])
		rospy.loginfo(msg)
		mouseToJoyPublisher.publish(msg)
		#rate.sleep()


if __name__ == '__main__':
	mouseToJoy()
