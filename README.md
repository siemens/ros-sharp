# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

## Recent Changes ##

[This](https://github.com/siemens/ros-sharp/commit/acdd1ea7b8de47a23fbf376fa590590cf945b495) commit comes with major changes in how ROS# deals with URDF import/export

The biggest changes are:
* [Urdf Libary](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf): The UrdfImporter project was renamed to Urdf. It now supports the ability to both read from and write to URDF files.
* [Create, Modify, and Export URDF models in Unity](https://github.com/siemens/ros-sharp/tree/master/Unity3D): ROS# now supports creating and exporting URDF models directly in Unity. It is also possible to modify and re-export an existing URDF model.
* [Transfer URDF files from Unity to ROS](https://github.com/siemens/ros-sharp/wiki/User_App_ROS_TransferURDFToROS): Previously it was only possible to transfer/import URDF files from ROS to Unity. Now ROS# can send a URDF and all its meshes from Unity to a package in ROS.

Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki/), especially [Section 3.2](https://github.com/siemens/ros-sharp/wiki/User_App_NoROS_ExportURDFOnWindows), for an explanation of how to use the new framework.

## Contents ##

* [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries):
 .NET solution for [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient), [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS):  [ROS](http://wiki.ros.org/) packages used by ROS#.
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): [Unity](https://unity3d.com/) project containing
  * Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and
   [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter)
  * example scenes and reference code (see [Wiki](https://github.com/siemens/ros-sharp/wiki))

## Releases ##

In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported in other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
  * to be used in other .NET projects using these libraries.

The latest release is also being published in the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-ros-unity-communication-package-107085).

Please get the development version with latest changes and fixes directly from the [tip of this master branch](https://github.com/siemens/ros-sharp).

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:
* `websocket-sharp.dll` from [websocket-sharp](https://github.com/sta/websocket-sharp) provided under MIT License (required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)).
* `Newtonsoft.Json.dll` from [NewtonSoft Json.Net](http://www.newtonsoft.com/json) provided under MIT License.

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on Linux and iOS by community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) solution requires .NET Framework 4.6 and Visual Studio 2017 to compile.
* The Unity Project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) requires Unity Version 2018.3 and higher.
Make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* Please find a UWP version of ROS# [here](https://github.com/dwhit/ros-sharp).
* Please find a .NET Standard 2.0 version of UrdfImporter [here](https://github.com/blommers/UdrfImporter).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
