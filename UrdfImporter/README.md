# [UrdfImporter](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter) #
... consists of:

## [UrdfSharp](https://github.com/analogtwin/ros-sharp/tree/master/UrdfImporter/UrdfSharp) ##
...  an import interface for [URDF](http://wiki.ros.org/urdf) files for .NET Applications.

## [UrdfSharpUnity3D](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter/UrdfSharpUnity3D) ##
...  Unity3D-specific extensions to [UrdfSharp](https://github.com/analogtwin/ros-sharp/tree/master/UrdfImporter/UrdfSharp) for importing [URDF](http://wiki.ros.org/urdf) in Unity3D.

# Dependencies #

* [UrdfSharp](https://github.com/analogtwin/ros-sharp/tree/master/UrdfImporter/UrdfSharp) has no external dependencies.
* [UrdfSharpUnity3D](https://github.com/siemens/ros-sharp/tree/master/UrdfImporter/UrdfSharpUnity3D) requires:
    1. Unity3D libraries
    2. [Websocket Sharp](https://github.com/sta/websocket-sharp): `websocket-sharp.dll`
    3. [NewtonSoft Json.Net](http://www.newtonsoft.com/json): `NewtonSoft.Json.dll`
    4. [ROSBridgeClient](https://github.com/siemens/ros-sharp/tree/master/ROSbridgeClient): `RosBridgeClient.dll`

### Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki) for further info. ###

---

© Siemens AG, 2017

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
