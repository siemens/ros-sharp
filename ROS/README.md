# [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS) #
... contains code for [ROS](http://wiki.ros.org/) nodes and [ROS](http://wiki.ros.org/) launch files which are useful to [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient) and/or [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter).

## [file_server](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server)
... provides a [ROS](http://www.ros.org/) node with a service to send file contents.

 ... is required for receiving meshes and textures referenced in a  [URDF](http://wiki.ros.org/urdf) via [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter).

## [joy_to_vel](https://github.com/siemens/ros-sharp/tree/master/ROS/joy_to_vel)
... provides a [ROS](http://www.ros.org/) node subscribing a [Joy message](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html) and publishing corresponding [Twist message](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

... is useful for working with [VelocitySubscriber](https://github.com/siemens/ros-sharp/blob/master/Unity3D/Assets/RosSharp/Scripts/VelocitySubscriber.cs) and there is no other ROS node providing a `\cmd_vel` topic.

### Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info. ###

---

© Siemens AG, 2017

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
