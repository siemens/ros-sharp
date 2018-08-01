# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

## Notes On This Fork ##

This fork has implemented changes to the repo to enable building to UWP devices, like the Microsoft HoloLens. 

#### Installation ### 

To use ROS# with the HoloLens, simply clone this fork and stay on the master branch. Then open the Unity project and import the [Microsoft Mixed Reality Toolkit](https://github.com/Microsoft/MixedRealityToolkit-Unity). The toolkit provides a version of Newtonsoft.Json that works on the HoloLens, as well as other tools/features you will want during HoloLens development. Follow the Mixed Reality Toolkit configuration instructions, and you will be good to go. 

### Architecture ###

How does this work under the hood? In brief, I wrote a UWP-compatible WebSocket interface for ROS#, created a UWP-compatible version of RosBridgeClient.dll, called RosBridgeClientUWP.dll, added that to the Unity Project, specified proper platforms for all .dlls, and edited RosConnector.cs to automatically use to the UWP WebSocket interface.

#### Creating RosBridgeClientUWP.dll ####

ROS# contains a solution in the Libraries folder, which contains a project called RosBridgeClient. In the Protocols folder, I created a UWP compatible WebSocket interface. Next, I wrapped all WebSocket protocol files in preprocessor directives so only the UWP compatible interface would be compiled in a UWP-build. Finally, I created a second project in the solution called RosBridgeClientUWP. I made it a Windows Universal class library project, and copied all of the RosBridgeClient code over as links. Copying as links means that editing the code in one location changes it in both. Finally, build the solution.

#### Preparing the Unity Project ####

In the ROS# Unity project, I first installed the Mixed Reality Toolkit and followed their configuration instructions. Next, in RosSharp/Plugins, I deselected all platforms from Newtonsoft.Json, and excluded WSAPlayer from all others. Next, I copied over the RosBridgeClientUWP.dll into RosSharp/Plugins, and selected WSAPlayer as the only platform. Finally, I modified RosConnector.cs, using preprocessor directives to make Unity use the WebSocketUWP protocol if WINDOWS_UWP is defined.

### Making Changes ###

If you want to make changes to the RosBridgeClient, like adding new messages, for instance, simply edit the code in the RosBridgeClient project (following the instructions from the main ROS# wiki), build the solution, and copy over the new RosBridgeClient.dll and RosBridgeClientUWP.dll.


## Recent Changes ##

[This](https://github.com/siemens/ros-sharp/commit/34fb2a8ddd58c5f099b1e4b887a253b954808fb4) commit comes with major changes in [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) that already have been discussed in [this](https://github.com/siemens/ros-sharp/issues/59) issue.

The biggest changes are:
* [Generic Communication Protocol Interface](https://github.com/siemens/ros-sharp/wiki/Dev_Protocols): an [interface](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/IProtocol.cs) to the communication protocol used by RosBridgeClient. It currently comes with two implementations: [WebSocketNetProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketNetProtocol.cs) and [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)
* [Message Type Structure](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Messages): a more ROS-oriented structure of message types, including further code simplifications
* [Simplified Message Handling in Unity](https://github.com/siemens/ros-sharp/wiki/Dev_MessageHandlingCodeMap.pdf): a simplified structure with even better performance. It requires a different placement of ROS# components in Unity scene.

Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki/) and the videos of [UnitySimulationScene](https://youtu.be/Ctv4BioS1Y0) and [GazeboSimulationScene](https://youtu.be/oh4BIE5qKoM) for a detailed info on how to use the new framework.

If you prefer working with the old framework, please revert to  [this](https://github.com/siemens/ros-sharp/commit/672b428b958456b20cb8b4f8b66afa720a3a435a) commit or to the corresponding [release v1.2c](https://github.com/siemens/ros-sharp/releases/tag/v1.2c).

## Contents ##

* [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries):
 .NET solution for [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient), [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter)
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS):  [ROS](http://wiki.ros.org/) packages used by ROS#.
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): [Unity](https://unity3d.com/) project containing
  * Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and
   [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter)
  * example scenes and reference code (see [Wiki](https://github.com/siemens/ros-sharp/wiki))

## Releases ##

In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported in other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter)
  * to be used in other .NET projects using these libraries.

The latest release is also being published in the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-ros-unity-communication-package-107085).

Please get the development version with latest changes and fixes directly from the [tip of this master branch](https://github.com/siemens/ros-sharp).

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:
* `websocket-sharp.dll` from [websocket-sharp](https://github.com/sta/websocket-sharp) provided under MIT License (required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)).
* `Newtonsoft.Json.dll` from [NewtonSoft Json.Net](http://www.newtonsoft.com/json) provided under MIT License.

[UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter) requires:
* `MathNet.Numerics.dll` from [Math.NET Numerics](https://numerics.mathdotnet.com/) provided under MIT License.

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on Linux and iOS by community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) solution is built using .NET Framework 4.6. It requires Unity 2018.x to run. Make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* [David Whitney](https://github.com/dwhit) provides a UWP version of ROS# on [his fork](https://github.com/dwhit/ros-sharp/tree/WebSocketUWP).

* Additionally [Blommers](https://github.com/blommers) kindly provides a [.NET Standard 2.0 version of UrdfImporter](https://github.com/blommers/UdrfImporter).

## ROS# Project Team ##

* [Martin Bischoff](https://github.com/MartinBischoff)
* [Berkay Alp Cakal](https://github.com/berkayalpcakal)
* [Verena Roehrl](https://github.com/roehrlverena)
* [Suzannah Smith](https://github.com/SuzannahSmith)

### Former Members and Visitors ###

* [Rahul Warrier](https://github.com/jaguar243)


## Special Thanks ##

* [Karl Henkel](https://github.com/karl-) for providing the [reference](https://github.com/karl-/pb_Stl) for the Unity STL mesh importer used in this project.
* [Jeremy Fix](https://github.com/jeremyfix) for providing some helpful ROS communication example scripts in Unity.
* [Hassanbot](https://github.com/hassanbot) for multiple bugfixes and [communication performance tests](https://github.com/siemens/ros-sharp/issues/66).
* [David Whitney](https://github.com/dwhit) and  [tarukosu](https://github.com/tarukosu-) for UWP platform support.
* [Interested in contributing as well?](CONTRIBUTING.md)

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
