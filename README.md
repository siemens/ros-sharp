# [<img src="https://github.com/siemens/ros-sharp/wiki/img/RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

... is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity3D](https://unity3d.com/).

## Contents: ##

### [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient) ###
... provides a [.NET](https://www.microsoft.com/net) API to [ROS](http://www.ros.org/) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite).

### [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter) ###
... provides a [URDF](http://wiki.ros.org/urdf) file parser for [.NET](https://www.microsoft.com/net) applications.

### [file_server](https://github.com/siemens/ros-sharp/tree/master/file_server) ###
... provides a [ROS](http://www.ros.org/) node with a service to send file contents.

 ... is required for receiving meshes and textures referenced in a  [URDF](http://wiki.ros.org/urdf) via [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter).
 
### [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) ###
... is a [Unity3D](https://unity3d.com/) reference project providing [Unity3D](https://unity3d.com/)-specifc extensions to 
* [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient)
* [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter)

### [Release](https://github.com/siemens/ros-sharp/tree/master/Release) ###
... contains [RosSharp.unitypackage](https://github.com/siemens/ros-sharp/tree/master/Release/RosSharp.unitypackage)
the latest package of [ROS#](https://github.com/siemens/ros-sharp) assets from the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project.

### [Tutorials](https://github.com/siemens/ros-sharp/tree/master/Tutorials) ###
... contains Unity3D tutorial projects described described in the [Wiki](https://github.com/siemens/ros-sharp/wiki).

## Licensing: ##

ROS# is open source under the Apache 2.0 license and is free for commercial use.

## External Dependencies: ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient) uses the following 3rd party libraries:
* `websocket-sharp.dll` from [websocket-sharp](https://github.com/sta/websocket-sharp) provided under MIT License.
* `NewtonSoft.Json.dll` from [NewtonSoft Json.Net](http://www.newtonsoft.com/json) provided under MIT License.

[UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter) uses the following 3rd party libraries:
* `MathNet.Numerics.dll` from [Math.NET Numerics](https://numerics.mathdotnet.com/) provided under MIT License.
* `System.Threading.dll` from [TaskParallelLibrary for .NET 3.5](https://www.nuget.org/packages/TaskParallelLibrary/1.0.2856) provided under [MS-EULA License](https://msdn.microsoft.com/en-us/hh295787).

## .NET Standard 2.0: ##
Both [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient)
and [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter)
are running on .NET Framework 3.5 as this is the .NET platform currently supported by official Unity3D releases.

For Non-Unity3D-Applications [blommers](https://github.com/blommers) kindly provides a
[.NET Standard 2.0 version of UrdfImporter](https://github.com/blommers/UdrfImporter).

## Special Thanks: ##

* [Rahul Warrier](https://github.com/jaguar243) for adjusting the code to enable its open source publication.
* [Verena Roehrl](https://github.com/roehrlverena) for providing the Wiki pages and the tutorial projects.
* [Karl Henkel](https://github.com/karl-) for providing the [reference](https://github.com/karl-/pb_Stl) for the Unity3D STL mesh importer used in this project.

### Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info. ###

---

© Siemens AG, 2017

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
