# [![ROS#](https://github.com/siemens/ros-sharp/wiki/img/ROSsharpLogo.jpg "ROS#")](https://github.com/siemens/ros-sharp) #
... is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular  [Unity3D](https://unity3d.com/).

## Contents: ##

### [ROSbridgeClient](https://github.com/siemens/ros-sharp/tree/master/ROSbridgeClient) ###
... provides a [.NET](https://www.microsoft.com/net) API to [ROS](http://www.ros.org/) via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite).

### [URDFsharp](https://github.com/siemens/ros-sharp/tree/master/URDFsharp) ###
... is an import interface for [URDF](http://wiki.ros.org/urdf) files.

### [file_server](https://github.com/siemens/ros-sharp/tree/master/file_server) ###
... provides a [ROS](http://www.ros.org/) node with a service to send file contents. [file_server](https://github.com/siemens/ros-sharp/tree/master/file_server) is required for accessing meshes and textures referenced in a  [URDF](http://wiki.ros.org/urdf) file from [ROS](http://www.ros.org/).

### Please see the Wiki for further info. ###

## Issues: ##

1. In `URDFSharp/URDFImporter/OdomoteryPatcher.cs`, the name of the root node needs to be generalized - either pull it from the urdf, or specify as a requirement that the root node be named "world".
2. In `URDFSharp/URDFImporterEditor/UrdfImporterEditorWindow.cs`, `urdfAssetPath` needs to be generalized using full path to Project folder (Windows method of finding current folder does not work in Mac OSX).

---

© Siemens AG, 2017

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
Revised: Rahul B. Warrier (10/21/2017)
