# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

## Overview ##

[ROS#](https://github.com/siemens/ros-sharp) is a set of open-source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/). With ROS#, developers can effortlessly create .NET applications that communicate with ROS nodes, subscribe to and publish topics, handle actions and services, and interact with ROS messages. This enables the development of robotics applications, simulations, and automation systems within the .NET ecosystem.

[Here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases) are some showcases illustrating what can be done with ROS#. The community provided various other application examples for ROS# [here](https://github.com/siemens/ros-sharp/issues/20). Please don't hesitate to post yours!

## Installation ##

ROS# can be used with Unity by installing the package or with any compatible (please see [platform support](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#platform-support) and [external dependencies](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#external-dependencies)) .NET project by integrating the libraries (RosBridgeClient, MessageGeneration, Urdf).
### For Unity Integration ###

+ #### Using Unity Package Manager (Recommended) ####

  1. Open Unity Package Manager.
  2. Click the "Add package from git URL" option at the top left corner.
  3. Paste the following URL:
  https://github.com/siemens/ros-sharp.git?path=/com.siemens.ros-sharp
  
  4. This method allows you to access the latest updates more quickly.

+ #### Unity Asset Store ####

  1. Download the package from the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-107085).

+ #### Manual Installation from Releases (Offline) ####

  1. Download the desired release from the [releases](link-to-releases) section.
  2. Open Unity Package Manager.
  3. Click the "Add package from disk" option at the top left corner.
  4. Select `com.siemens.ros-sharp/package.json` from the downloaded release (only for ROS# version > 2.0.0).

### For .NET Projects ###

+ #### Cloning the Repository (Recommended) ####

  1. Clone this repository to your local machine.
  2. Integrate the necessary libraries into your .NET project.

+ #### NuGet Installation ####

  1. Head to the [NuGet](https://www.nuget.org/profiles/MartinBischoff) page.
  2. Install the required packages individually from NuGet.

+ #### Manual Installation from Releases (Offline) ####

  1. Download the desired release from the [releases](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#releases) section.
  2. Integrate the libraries into your .NET project as needed.

## Getting Started ##

#### Setting Up ROS Websocket ####

ROS# uses websocket (for now). So you need to install and launch a ROS 1 or ROS 2 websocket on your system. Here's how:

__For ROS1__

```bash
# Install rosbridge_suite
sudo apt-get install ros-noetic-rosbridge-suite

# Launch ROS 1 websocket
roslaunch rosbridge_server rosbridge_websocket.launch
```
__For ROS2__
```bash
# Install rosbridge_suite
sudo apt-get install ros-humble-rosbridge-server

# Launch ROS 2 websocket
ros2 launch rosbridge_server rosbridge_websocket.launch
```

>__Note:__ ROS#, Unity, and .NET projects: Ensure that ROS# and your ROS websocket server run on the same local network. You can find your local IP address using the command ```hostname -I``` in the Ubuntu console.

#### Setting Up ROS# For Unity Projects ####

1. Make sure [external dependencies](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#external-dependencies) are satisfied and you're working on a [supported platform](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#external-dependencies).
2. Create an empty GameObject in your Unity scene.
3. Add the RosSocket script to the GameObject.
4. Fill in your local address and select your desired serializer and protocol. A specific serializer/protocol might not work for your other packages. If both serializers/protocols are not working for your needs, head to making a custom serializer/protocol. If Unity doesn't complain, leave it as is.
5. Run your Unity project. If you see a 'connected' notification, you've successfully connected to ROS#.

Congratulations! You've made your first ROS# connection. Proceed to the [beginner tutorial](TODO) to learn about message publishing/subscribing within Unity.

#### Setting Up ROS# For .NET Solutions ####

1. Make sure [external dependencies](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#external-dependencies) are satisfied and you're working on a [supported platform](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#external-dependencies).
2. Refer to the [RosCommunicatonTests](https://github.com/siemens/ros-sharp/blob/8330f5b433da22fff028dd1f355233b2f3e008e7/Libraries/RosBridgeClientTest/RosCommunicationTests.cs) to see how a RosSocket can be initialized within a C# script.
3. (TODO) Head to this [beginner tutorial](TODO) for further explanation.

For more detailed installation information, please refer to [wiki](https://github.com/siemens/ros-sharp/wiki).

## External Dependencies ##

#### .NET Solution ####
- [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:

  * [Newtonsoft.Json](https://github.com/JamesNK/Newtonsoft.Json) [13.0.3] (MIT License)
  * [websocket-sharp](https://github.com/sta/websocket-sharp) (MIT License), required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs) ???
  * Microsoft.Bcl.AsyncInterfaces [8.0.0]
  * System.Data.DataSetExtensions [4.0.5]
  * System.Runtime.CompilerServices.Unsafe [6.0.0]
  * System.Text.Encodings.Web [8.0.0]
  * System.Text.Json [8.0.0]
  * System.Threading.Channels [8.0.0]
  * WebSocketSharp-netstandard [1.0.1]
  
- [MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration) requires:
  * System.Data.DataSetExtensions [4.0.5]
  
- [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/urdf) requires:
  * System.Data.DataSetExtensions [4.0.5]

#### Unity Package ####
There is no need to install them manually; they are already included in the package (**`com.siemens.ros-sharp/Plugins`**). Versions of these dependencies are different than .NET solution dependencies. Most of the new versions conflict with Unity's built-in dlls. If you really need to know, you can use a dll analyzer tool to learn their specific versions.
* Newtonsoft.Json
* websocket-sharp
* Microsoft.Bcl.AsyncInterfaces
* System.Runtime.CompilerServices.Unsafe
* System.Text.Encodings.Web
* System.Text.Json
* System.Threading.Channels

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on various other platforms by community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) Visual Studio solution requires .NET 8 and Visual Studio 2022 or higher.
* The [Unity package] (https://github.com/siemens/ros-sharp/tree/master/com.siemens.ros-sharp) has been developed with Unity Version 2022.3.17f1 (LTS) and should also be compatible with older versions.
In Versions below 2022.3, make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

## Project Structure ##

The ROS# project follows a specific folder structure to organize its components and resources. Below is an overview of the main directories and their purposes:

- **`com.siemens.ros-sharp/`**: Contains custom Unity [UPM](https://docs.unity3d.com/Manual/CustomPackages.html) package for integrating ROS# into Unity projects. The package includes the whole ROS# .NET solution, as well as Unity specific scripts, external dependencies, and samples. This folder follows a planned layout as Unity [recommends](https://docs.unity3d.com/Manual/cus-layout.html).
  - **`Runtime/`**: Core ROS# .NET scripts and components for Unity integration.
  - **`Editor/`**: Unity specific editor scripts for mainly UI extensions.
  - **`Plugins/`**: External dependencies with specific versions.
  - **`Samples~/`**: Example samples that need to be imported through the package manager window. For more info about example scenes and reference code, see [wiki](https://github.com/siemens/ros-sharp/wiki).
  - **`Documentation/`**: Package documentation.

- **`Libraries/`**: .NET solution containing the core ROS# libraries and tools for communicating with ROS.
  - **`RosBridgeClient/`**: Core ROS# .NET library for communicating with ROS via websockets.
  - **`MessageGeneration/`**: Tools for generating C# classes from ROS message definitions, including messages, actions, and services with both ROS1/2 support.
  - **`Urdf/`**: Library for parsing URDF files and creating Unity GameObjects.
  - **`PostBuildEvents/`**: Windows (powershell) and Linux (bash) scripts are used to automatically update the Unity package after each successful .NET build. If you are not modifying the core ROS# .NET scripts while testing/working with the Unity package simultaneously, there is no need for concern. Please see [post build events](TODO) for more information.

- **`ROS Packages/`**: ROS [packages](https://github.com/siemens/ros-sharp/tree/8330f5b433da22fff028dd1f355233b2f3e008e7/ROS) used by ROS# for testing and demonstration purposes.

## Releases (should we delete???) ##

In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported into other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
  * to be used in other .NET projects using these libraries.

The latest release is also being published in the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-ros-unity-communication-package-107085).

Please get the latest development version directly from the [tip of this master branch](https://github.com/siemens/ros-sharp).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2024

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
