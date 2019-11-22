# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

[Here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases) are some showcases illustrating what can be done with ROS#.
Community provided a variety of other application examples for ROS# [here](https://github.com/siemens/ros-sharp/issues/20). Please don't hesitate to post yours!

## Recent Changes ##

#### Automatic Message Generation: ####

With ROS# you can now generate C# classes of any ROS message, service or action type.

For example, the auto-generated class of [sensor_msgs/JointState.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) is [JointState.cs](https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/MessageTypes/Sensor/msg/JointState.cs).

Please find the new library [here](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration) and the console tool sources [here](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGenerationConsoleTool).
Please also note the new Unity Editor menu features.

Check out the new [Wiki page](https://github.com/siemens/ros-sharp/wiki/Dev_NewMessageTypes) for further info.

#### Namespace Changes: ####

We had to update the namespaces as follows:

| old                                            | new                                         |
|------------------------------------------------|---------------------------------------------|
| `RosSharp.RosBridgeClient.Messages`            |`RosSharp.RosBridgeClient.MessageTypes`      |
|`RosSharp.RosBridgeClient.Services`             | `RosSharp.RosBridgeClient.MessageTypes`     |
| `RosSharp.RosBridgeClient.Messages.Standard`   | `RosSharp.RosBridgeClient.MessageTypes.Std` |
| `RosSharp.RosBridgeClient.Messages.Navigation` | `RosSharp.RosBridgeClient.MessageTypes.Nav` |

## Contents ##

* [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries): .NET solution for
[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient),
[Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf) and
[MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration)
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS):  [ROS](http://wiki.ros.org/) packages used by ROS#.
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): [Unity](https://unity3d.com/) project containing
  * Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and
   [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter) and
   [MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration)
  * example scenes and reference code (see [Wiki](https://github.com/siemens/ros-sharp/wiki))

## Releases ##

In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported in other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
  * to be used in other .NET projects using these libraries.

The latest release is also being published in the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-ros-unity-communication-package-107085).

Please get the latest development version directly from the [tip of this master branch](https://github.com/siemens/ros-sharp).

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:
* [Newtonsoft.Json](https://github.com/JamesNK/Newtonsoft.Json) (MIT License)
* [Newtonsoft.Json.Bson](https://github.com/JamesNK/Newtonsoft.Json.Bson) (MIT License)
* [websocket-sharp](https://github.com/sta/websocket-sharp) (MIT License), required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on a variety of other platforms community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) Visual Studio solution requires .NET Framework 4.6 and Visual Studio 2017.
* The Unity Project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) requires Unity Version 2018.3 and higher.
In Versions below 2019.3, make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* Please find a UWP version of ROS# [here](https://github.com/dwhit/ros-sharp).
* Please find a .NET Standard 2.0 version of UrdfImporter [here](https://github.com/blommers/UdrfImporter).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2019

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
