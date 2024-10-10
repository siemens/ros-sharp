#!/usr/bin/env python3

# © Siemens AG, 2024
# Author: Mehmet Emre Cakal (emre.cakal@siemens.com)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import tkinter as tk

class MouseToJoy(Node):
    def __init__(self):
        super().__init__('mouse_to_joy2')
        
        self.msg_period = 33

        # Setup MouseToJoy Publisher 
        self.mouse_to_joy_publisher = self.create_publisher(Joy, 'joy', self.msg_period)

        # Initialize axes values
        self.vel_linear = 0.0
        self.vel_angular = 0.0

        # Initialize Tkinter window
        self.root = tk.Tk()
        self.root.title('Mouse to Joy')
        self.root.geometry('800x800')
        self.root.resizable(False, False)

        # Main frame
        self.main_frame = tk.Frame(self.root, width=800, height=800, bg="#ffffff")
        self.main_frame.pack_propagate(False)
        self.main_frame.pack()
        
        # Text labels
        move_label = tk.Label(self.main_frame, text='Move your cursor here!', font=('Helvetica', 22), bg="#ffffff")
        move_label.place(relx=0.5, rely=0.5, anchor='center')

        ros_label = tk.Label(self.main_frame, text='ROS#', font=('Helvetica', 18), bg="#ffffff")
        ros_label.place(relx=0.02, rely=0.98, anchor='sw')

        siemens_label = tk.Label(self.main_frame, text='Siemens', font=('Helvetica', 18), fg="#009999", bg="#ffffff")
        siemens_label.place(relx=0.98, rely=0.98, anchor='se')

        # Canvas
        self.main_frame.bind('<Motion>', self.on_mouse_motion)

        # Schedule periodic callback for publishing Joy messages
        self.publish_joy_message()

        # Start Tkinter main loop (thread)
        self.root.mainloop()

    def on_mouse_motion(self, event):
        # Convert mouse position to velocity
        canvas_width = self.main_frame.winfo_width()
        canvas_height = self.main_frame.winfo_height()
        self.vel_linear = (event.y - canvas_height / 2) / (canvas_height / 2)
        self.vel_angular = (event.x - canvas_width / 2) / (canvas_width / 2)

    def publish_joy_message(self):
        # Publish Joy message based on current mouse position
        msg = Joy()
        msg.axes = [self.vel_linear, self.vel_angular]
        msg.buttons = []

        # Log and Publish 
        self.get_logger().info(f'Joy message: {msg}')
        self.mouse_to_joy_publisher.publish(msg)

        # Schedule next callback
        self.root.after(self.msg_period, self.publish_joy_message)

def main(args=None):
    rclpy.init(args=args)
    node = MouseToJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.root:
            node.root.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
