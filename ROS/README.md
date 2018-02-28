# [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS) #
contains the following  [ROS](http://wiki.ros.org/) packages:

* [file_server](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server)
  * Launch [file_server.launch](https://github.com/MartinBischoff/ros-sharp/blob/master/ROS/file_server/launch/file_server.launch) to provide the ROS service `/file_server/get_file` for sending file contents.
 This service  is called by [UrdfIUmporter.cs](https://github.com/siemens/ros-sharp/blob/master/RosBridgeClient/UrdfImporter.cs) to receive URDF resource  files (meshes and textures) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite).
  * Launch [publish_description_turtlebot2.launch] (https://github.com/MartinBischoff/ros-sharp/blob/master/ROS/file_server/launch/publish_description_turtlebot2.launch) to provide the ROS `/robot_description` parameter for [Turtlebot2](http://wiki.ros.org/Robots/TurtleBot).

* [gazebo_simulation_scene](https://github.com/siemens/ros-sharp/tree/master/ROS/gazebo_simulation_scene)
  * Launch ``gazebo_simulation_scene.launch`` to initialize ROS for the Gazebo Simulation Scene.

* [unity_simulation_scene](https://github.com/siemens/ros-sharp/tree/master/ROS/unity_simulation_scene)
  * Launch ``unity_simulation_scene`` to initialize ROS for the Unity Simulation Scene.

__Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info.__

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
