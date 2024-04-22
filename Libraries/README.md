# [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries) #
 the .NET solution [RosSharp.sln](https://github.com/siemens/ros-sharp/tree/master/Libraries/RoRosSharp.sln) consists of the following projects:

 * [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient): .NET API to [ROS](http://www.ros.org/) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
 * [RosBridgeClientTest](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClientTest): NUnit and Console Application tests for RosBridgeClient
  * [MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration): .NET Library for generating C# source code for ROS message/service/action.
 * [MessageGenerationConsoleTool](https://github.com/ros-sharp/ros-sharp/tree/master/Libraries/MessageGenerationConsoleTool): Command line tool for generating C# source code for ROS message/service/action.
 * [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf): URDF file parser for .NET applications

If you are coming from an older (<2.0.0) version of ROS#, libraries `MessageGeneration.dll`, `RosBridgeClient.dll` and `UrdfImporter.dll` were used to required [plugins](https://github.com/siemens/ros-sharp/Unity3D/Assets/RosSharp/Plugins/) for the Unity project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D). After ROS# version 2.0.0, we only imported the source code and not the dll files because Unity 22 LTS encountered problems with the Visual Studio build dlls due to its outdated C# dependencies. In this way, Unity can generate the ROS# dlls itself according to the assembly definition files within the Unity package without compatibility errors.  

__Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info.__

---

© Siemens AG, 2017-2024

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
