# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

[Here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases) are some showcases illustrating what can be done with ROS#.
Community provided a variety of other application examples for ROS# [here](https://github.com/siemens/ros-sharp/issues/20). Please don't hesitate to post yours!

## Notes On This Fork ##

This fork has some changes to allow ROS# to be used in UWP projects, such as the Microsoft HoloLens (1 & 2 - tested and deployed successfully). Like the main ROS# branch, use 2019.x or later.

## How to get started ##

Step 1: Clone this project

Step 2, Option 1: Open the [Demo Project](https://github.com/EricVoll/ros-sharp/tree/master/ProjectSetup/RosSharpUnity) and copy its content. It is setup in a way, that it is buildable for UWP applications and should work right out of the box.

Step 2, Option 2: Follow these instructions:
 - Create a new Unity Project or use an existing one
 - Copy the [RosSharp folder](https://github.com/EricVoll/ros-sharp/tree/master/Unity3D/Assets) into the Assets Folder of your project
 - Configure the RosBridgeClientUWP.dll to be used for the WSA platform and disable it for all other platforms
 - Configure the RosBridgeClient.dll to be excluded for the WSA platform.
 - Copy the [NewtonSoft AOT version](https://github.com/EricVoll/ros-sharp/tree/master/ProjectSetup/RosSharpUnity/Assets/Plugins) into your project (e.g. Assets/Plugins) You don't have to configure any targetplatforms as with the RosBridgeclient(UWP) dlls, since they should be configured correctly. The original source of this is [here](https://github.com/jilleJr/Newtonsoft.Json-for-Unity)

It is important that the [link.xml file](https://github.com/EricVoll/ros-sharp/blob/master/Unity3D/Assets/RosSharp/link.xml) is somewhere within the /Assets/ folder of the Unity Project. The reasons for that are described [here](https://github.com/jilleJr/Newtonsoft.Json-for-Unity/wiki/Fix-AOT-using-link.xml). Basically, Unity's IL2CPP compiler strips away unused code. Since RosSharp works with classes that are never really used but only serialized to be sent via WebSockets, many of these "unused" fields and properties are removed, resulting in (almost) empty json-strings. The link.xml file tells Unity's IL2CPP compiler to not strip any code from the specified assemblies.

With this setup your project should work in Editor Mode and in UWP-Mode.

The simple Demo-App subscribes to the /clock topic and displays the values received. This was successfully deployed to HoloLens 2
![deployed RosSharp UWP to HoloLens2](https://www.github.com/EricVoll/ros-sharp/wiki/HoloLens.jpg)

## How it works ##
There are four main changes to the original Ros-Sharp repository
1. [AOT compilability](https://en.wikipedia.org/wiki/Ahead-of-time_compilation)
The original RosSharp uses reflection and the normal NewtonSoft Json library, which are not AOT compilable and fail to work after Unity's IL2CPP compiler process the files. There were some changes made to the library to not use reflection, and also a AOT compilable version of NewtonSoft Json is included in the Demo-Project
2. Websockets
Prev. another than the default version for Websockets was needed, but the newer version of WebSocketSharp works fine for UWP projecst
3. UWP DLL
The DLL was created by creating a UWP class library project, and then linking all folders from the original library. The including part is done in the project file. Obj, Bin and Properties folders were of course excluded.
4. Compiler constants / defines
Some parts of the library and Unity Scripts needed some platform specific code, which was done with the "WINDOWS_UWP" compiler constant.

### Compatibile With Mixed Reality Toolkit ###
This branch is compatible with Microsoft's Mixed Reality Toolkit. See the Preparing Unity Project Section.

Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki/), especially [Section 3.2](https://github.com/siemens/ros-sharp/wiki/User_App_NoROS_ExportURDFOnWindows), for an explanation of how to use the new framework.

## Contents ##

* [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries): .NET solution for
[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient),
[RosBridgeClientUWP](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClientUWP),
[Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf) and
[MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration)
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS):  [ROS](http://wiki.ros.org/) packages used by ROS#.
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): [Unity](https://unity3d.com/) project containing
  * Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and
   [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter) and
   [MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration)
  * example scenes and reference code (see [Wiki](https://github.com/siemens/ros-sharp/wiki))
## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and UWP requires:
* [Newtonsoft.Json](https://github.com/JamesNK/Newtonsoft.Json) (MIT License)
* [Newtonsoft.Json.Bson](https://github.com/JamesNK/Newtonsoft.Json.Bson) (MIT License)
* [websocket-sharp](https://github.com/sta/websocket-sharp) (MIT License), required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on a variety of other platforms community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) Visual Studio solution requires .NET Framework 4.6 and Visual Studio 2017 or higher.
* The Unity Project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) requires Unity Version 2018.3 or higher.
In Versions below 2019.3, make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* Please find a UWP version of ROS# [here](https://github.com/EricVoll/ros-sharp).
* Please find a .NET Standard 2.0 version of UrdfImporter [here](https://github.com/blommers/UdrfImporter).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2020

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
