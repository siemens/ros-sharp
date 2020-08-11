# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

## Notes On This Fork ##

This fork has some changes to allow ROS# to be used in UWP projects, such as the Microsoft HoloLens. Like the main ROS# branch, use 2019.x or later.

## How to get started ##

Step 1: Clone this project

Step 2, Option 1: Open the [Demo Project](https://github.com/EricVoll/ros-sharp/tree/master/ProjectSetup/RosSharpUnity) and copy its content. It is setup in a way, that it is buildable for UWP applications.

Step 2, Option 2: Follow these instructions:
 - Create a new Unity Project or use an existing one
 - Copy the [RosSharp folder](https://github.com/EricVoll/ros-sharp/tree/master/Unity3D/Assets) into the Assets Folder of your project
 - Configure the RosSharpClientUWP.dll to be used for the WSA platform and disable it for all other platforms
 - Configure the RosSharpClient.dll to be excluded for the WSA platform.
 - Configure the NewtonSoft.dll to be excluded from ALL platforms
 - Copy the [NewtonSoft AOT version](https://github.com/EricVoll/ros-sharp/tree/master/ProjectSetup/RosSharpUnity/Assets/Plugins) into your project (e.g. Assets/Plugins)

It is important to keep the NewtonSoft.dll in the RosSharp/Plugins folder, to "trick" unity during development time that a compatible version of NewtonSoft is present. This version is not AOT compilable and will not work in UWP applications using the IL2CPP backend. When building the "Game" Unity ignores this "faulty" version of NewtonSoft and uses the AOT-Version of NewtonSoft instead, which will work.

With this setup your project should work in Editor Mode and in UWP-Mode.

### Architecture ###

How does this work under the hood? In brief, I wrote a UWP-compatible WebSocket interface for ROS#, created a UWP-compatible version of RosBridgeClient.dll, called RosBridgeClientUWP.dll, added that to the Unity Project, specified proper platforms for all .dlls, and edited RosConnector.cs to automatically use to the UWP WebSocket interface.

#### Creating RosBridgeClientUWP.dll ####

ROS# contains a solution in the Libraries folder, which contains a project called RosBridgeClient. In the Protocols folder, I created a UWP compatible WebSocket interface. Next, I wrapped all WebSocket protocol files in preprocessor directives so only the UWP compatible interface would be compiled in a UWP-build. Finally, I created a second project in the solution called RosBridgeClientUWP. I made it a Windows Universal class library project, and copied all of the RosBridgeClient code over as links. Copying as links means that editing the code in one location changes it in both. Finally, I built the solution.

### Making Changes ###

If you want to make changes to the RosBridgeClient, like adding new messages, for instance, simply edit the code in the RosBridgeClient project (following the instructions from the main ROS# wiki), build the solution, and copy over the new RosBridgeClient.dll and RosBridgeClientUWP.dll.


### Compatibile With Mixed Reality Toolkit ###
This branch is compatible with Microsoft's Mixed Reality Toolkit. See the Preparing Unity Project Section.

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

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:
* `websocket-sharp.dll` from [websocket-sharp](https://github.com/sta/websocket-sharp) provided under MIT License (required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)).
* `Newtonsoft.Json.dll` from [NewtonSoft Json.Net](http://www.newtonsoft.com/json) provided under MIT License.

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on Linux and iOS by community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) solution requires .NET Framework 4.6 and Visual Studio 2017 to compile.
* The Unity Project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) requires Unity Version 2018.2 and higher.
Make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* Please find a UWP version of ROS# [here](https://github.com/EricVoll/ros-sharp).
* Please find a .NET Standard 2.0 version of UrdfImporter [here](https://github.com/blommers/UdrfImporter).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
