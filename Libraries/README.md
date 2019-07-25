# [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries) #
 the .NET solution [RosSharp.sln](https://github.com/siemens/ros-sharp/tree/master/Libraries/RoRosSharp.sln) consists of the following projects:

 * [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient): .NET API to [ROS](http://www.ros.org/) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
 * [RosBridgeClientTest](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClientTest): NUnit and Console Application tests for RosBridgeClient
  * [MessageGeneration](https://github.com/sye8/ros-sharp/tree/master/Libraries/MessageGeneration): .NET Library for generating C# source code for ROS message/service/action.
 * [MessageGenerationMain](https://github.com/sye8/ros-sharp/tree/master/Libraries/MessageGenerationMain): Command line tool for generating C# source code for ROS message/service/action.
 * [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter): URDF file parser for .NET applications

Libraries `MessageGeneration.dll`, `RosBridgeClient.dll` and `UrdfImporter.dll` are required [plugins](https://github.com/siemens/ros-sharp/Unity3D/Assets/RosSharp/Plugins/) for the Unity project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D).

__Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info.__

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
