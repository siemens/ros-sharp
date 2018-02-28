# [<img src="https://github.com/siemens/ros-sharp/wiki/img/RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

## Contents ##

* [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient): a [.NET](https://www.microsoft.com/net) API to [ROS](http://www.ros.org/) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
* [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter): a [URDF](http://wiki.ros.org/urdf) file parser for [.NET](https://www.microsoft.com/net) applications
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS) some helpful [ROS](http://wiki.ros.org/) packages
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): a [Unity](https://unity3d.com/) project
providing Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient) and
   [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter)

## Releases ##
In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported in other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient) and [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter)
  * to be used in other .NET projects using these libraries.

Please get the latest development version directly from the [tip of the ROS# master branch](https://github.com/siemens/ros-sharp).

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient) uses the following 3rd party libraries:
* `websocket-sharp.dll` from [websocket-sharp](https://github.com/sta/websocket-sharp) provided under MIT License.
* `NewtonSoft.Json.dll` from [NewtonSoft Json.Net](http://www.newtonsoft.com/json) provided under MIT License.

[UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter) uses the following 3rd party libraries:
* `MathNet.Numerics.dll` from [Math.NET Numerics](https://numerics.mathdotnet.com/) provided under MIT License.
* `System.Threading.dll` from [TaskParallelLibrary for .NET 3.5](https://www.nuget.org/packages/TaskParallelLibrary/1.0.2856) provided under [MS-EULA License](https://msdn.microsoft.com/en-us/hh295787).

## .NET Standard 2.0 ##
Both [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/RosBridgeClient)
and [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter)
are running on .NET Framework 3.5 as this is the .NET platform currently supported by official Unity releases.

For Non-Unity-Applications [blommers](https://github.com/blommers) kindly provides a
[.NET Standard 2.0 version of UrdfImporter](https://github.com/blommers/UdrfImporter).

## Special Thanks ##

* [Rahul Warrier](https://github.com/jaguar243) for adjusting the code to enable its open source publication.
* [Verena Roehrl](https://github.com/roehrlverena) for providing  ROS packages, Wiki pages and Unity example scenes.
* [Karl Henkel](https://github.com/karl-) for providing the [reference](https://github.com/karl-/pb_Stl) for the Unity STL mesh importer used in this project.
* [Jeremy Fix](https://github.com/jeremyfix) for providing some helpful ROS communication example scripts in Unity.
* [Berkay Alp Cakal](https://github.com/berkayalpcakal)  for providing  ROS packages, Wiki pages and Unity example scenes.

* [Interested in contributing as well?](CONTRIBUTING.md)

## Further Info ##
* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki).
* [Contact the project team](mailto:ros-sharp.ct@siemens.com).

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
