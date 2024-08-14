[<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp)

## Overview ##

ROS# is a set of open-source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/). With ROS#, developers can effortlessly create .NET applications that communicate with ROS nodes, subscribe to and publish topics, handle actions and services, and interact with ROS messages. This enables the development of robotics applications, simulations, and automation systems within the .NET ecosystem.

<div style="text-align: center;">
  <img src="img_placeholder\Draft3_arch.png" alt="ROS# Showcase" style="padding-top: 5px; padding-bottom: 5px; width: 100%; height: 100%;">
</div>

[Here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases) are some showcases illustrating what can be done with ROS#. The community provided various other application examples for ROS# [here](https://github.com/siemens/ros-sharp/issues/20). 

<div style="text-align: center;">
<img src="img_placeholder\Draft3_showcase.png" alt="ROS# Showcase" style="padding-top: 5px; padding-bottom: 5px; width: 100%; height: 100%;">
</div>

## Installation ##

ROS# can be used with Unity Engine __and/or__ with any compatible .NET project, see [platform support](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#platform-support) and [external dependencies](https://github.com/siemens/ros-sharp/tree/master?tab=readme-ov-file#external-dependencies).

* ### For Unity Integration: Unity Package Manager ###
  1. In Unity Package Manager, click the "Add package from git URL" option at the top left corner.
  2. Paste the following URL: https://github.com/siemens/ros-sharp.git?path=/com.siemens.ros-sharp
  <img src="img_placeholder\User_Inst_InstallationROS_sharp_From_GitCombined.png" alt="ROS# Package Install from Git" style="padding-top: 10px; padding-bottom: 0px; width: 90%; height: 100%;">

> For more installation options, detailed instractions, and __getting started__ see wiki: [Installing and Configuring ROS# for Unity](https://github.com/siemens/ros-sharp/wiki/User_Inst_InstallationROS_sharp.md). 

* ### For .NET Projects: NuGet Gallery ###
  1. Head to the [NuGet](https://www.nuget.org/profiles/MartinBischoff) page.
  2. Install the required packages individually from NuGet.

> For more installation options, detailed instractions, and __getting started__ see wiki: [Installing and Configuring ROS# for .NET](https://github.com/siemens/ros-sharp/wiki/User_Inst_InstallationROS_sharp_DotNET.md). 


## Platform Support ##

* The [ROS#](https://github.com/siemens/ros-sharp/tree/master/Libraries/) dependencies require __.NET 8__ and __Visual Studio 2022__ or higher.
* The [Unity package](https://github.com/siemens/ros-sharp/tree/master/com.siemens.ros-sharp) has been developed with Unity Version __2022.3.17f1 (LTS)__ and should also be compatible with older versions.
> For Unity versions below 2022.3: See [wiki](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows).

## Repository Structure ##

Below is an overview of the main directories and their purposes:

- **`com.siemens.ros-sharp/`**: Contains the custom Unity [UPM](https://docs.unity3d.com/Manual/CustomPackages.html) package for integrating ROS# into Unity projects. The package includes the whole ROS# .NET solution, as well as Unity specific scripts, external dependencies, and samples. This folder follows a planned layout as Unity [recommends](https://docs.unity3d.com/Manual/cus-layout.html).
  - **`Runtime/`**: Core ROS# .NET scripts and components for Unity integration.
  - **`Editor/`**: Unity specific editor scripts for mainly UI extensions.
  - **`Plugins/`**: External dependencies with specific versions.
  - **`Samples~/`**: Example samples that need to be imported through the package manager window. For more info about example scenes and reference code, see [wiki](https://github.com/siemens/ros-sharp/wiki).

- **`Libraries/`**: .NET solution containing the core ROS# libraries and tools for communicating with ROS.
  - **`RosBridgeClient/`**: Core ROS# .NET library for communicating with ROS via websockets.
  - **`MessageGeneration/`**: Tools for generating C# classes from ROS message definitions, including messages, actions, and services with both ROS1/2 support.
  - **`Urdf/`**: Library for parsing URDF files and creating Unity GameObjects.
  - **`PostBuildEvents/`**: OS specific post build scripts. Please see [post build events](https://github.com/siemens/ros-sharp/wiki/Dev_PostBuildEvents.md) for more information.

- **`ROS Packages/`**: ROS [packages](https://github.com/siemens/ros-sharp/tree/8330f5b433da22fff028dd1f355233b2f3e008e7/ROS%20Packages) that are used by ROS# for tutorial, testing and demonstration purposes.



## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [External Dependencies](https://github.com/siemens/ros-sharp/wiki/Dev_ExternalDependencies)
* [Project Team, Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)
* [Contact the Project Team](mailto:ros-sharp.ct@siemens.com)

---
© Siemens AG, 2017-2024

