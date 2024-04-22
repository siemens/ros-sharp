# [ROS Packages](https://github.com/siemens/ros-sharp/tree/master/ROS) #
Contains the following [ROS](http://wiki.ros.org/) packages:

* [file_server](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server)
  * Launch [ros_sharp_communication.launch](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server/launch/ros_sharp_communication.launch) to launch the `rosbridge_server`, and to provide the ROS service `/file_server/get_file` for sending file contents.
 This service  is called by [UrdfIUmporter.cs](https://github.com/siemens/ros-sharp/blob/master/RosBridgeClient/UrdfImporter.cs) to receive URDF resource  files (meshes and textures) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite).
  * Launch [publish_description_turtlebot2.launch](https://github.com/siemens/ros-sharp/blob/master/ROS/file_server/launch/publish_description_turtlebot2.launch) to provide the ROS `/robot_description` parameter for [Turtlebot2](http://wiki.ros.org/Robots/TurtleBot).
  * Launch [visualize_robot.launch](https://github.com/siemens/ros-sharp/blob/master/ROS/file_server/launch/visualize_robot.launch) to visualize the received robot description in rViz with parameter `model:=$(find <package name>)/robot_description.urdf`

* [gazebo_simulation_scene](https://github.com/siemens/ros-sharp/tree/master/ROS/gazebo_simulation_scene)
  * Launch ``gazebo_simulation_scene.launch`` to initialize ROS for the Gazebo Simulation Scene.

* [unity_simulation_scene](https://github.com/siemens/ros-sharp/tree/master/ROS/unity_simulation_scene)
  * Launch ``unity_simulation_scene`` to initialize ROS for the Unity Simulation Scene.

Notice that these packages are made using ROS1 and have not been tested officially for ROS2. However, some of our valuable community members stated that they could run them with ROS2 as well.

__Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info.__

---

© Siemens AG, 2017-2024

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
