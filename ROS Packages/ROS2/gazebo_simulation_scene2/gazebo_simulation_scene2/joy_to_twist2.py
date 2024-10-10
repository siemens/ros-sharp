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
from geometry_msgs.msg import Twist

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist2')

        # Setup joy topic subscription
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.handle_joy_msg, 10)

        # Setup Twist Publisher
        self.twist_publisher = self.create_publisher(Twist, '/model/r2d2/cmd_vel', 10)

        self.scalers = [1.4, 1.4, 1.4, -6.28, -6.28, -6.28]

        
        # Initialize the latest Twist message
        self.latest_twist = Twist()

        # Initialize a timer to control the publish rate
        self.publish_rate = 30  # Hz
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_latest_twist)

    def handle_joy_msg(self, data):
        # self.get_logger().info('Received: %s' % str(data))

        # Start Mapping from Joy to Twist
        if len(data.axes) >= 6:
            self.latest_twist.angular.x = data.axes[5] * self.scalers[3]

        if len(data.axes) >= 5:
            self.latest_twist.linear.z = data.axes[4] * self.scalers[2]

        if len(data.axes) >= 4:
            self.latest_twist.angular.y = data.axes[3] * self.scalers[4]

        if len(data.axes) >= 3:
            self.latest_twist.linear.y = data.axes[2] * self.scalers[1]

        if len(data.axes) >= 2:
            self.latest_twist.angular.z = data.axes[1] * self.scalers[5]

        if len(data.axes) >= 1:
            self.latest_twist.linear.x = data.axes[0] * self.scalers[0]

    def publish_latest_twist(self):
        # self.get_logger().info('Publishing: %s' % str(self.latest_twist))
        self.twist_publisher.publish(self.latest_twist)

    def stop_robot(self):
        msg = Twist()
        self.twist_publisher.publish(msg)
        self.get_logger().info('Stopping Robot')

def main(args=None):
    rclpy.init(args=args)
    joy_to_twist = JoyToTwist()

    try:
        rclpy.spin(joy_to_twist)
    except KeyboardInterrupt:
        pass

    joy_to_twist.stop_robot()
    joy_to_twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
