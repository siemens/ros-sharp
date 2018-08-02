# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

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

* [David Whitney](https://github.com/dwhit) provides a UWP version of ROS# on [his fork](https://github.com/dwhit/ros-sharp).

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
