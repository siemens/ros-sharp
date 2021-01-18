# Differences between Unity Robotics Hub and ROS #

Below is a list of main differences between Unity Robotics Hub and ROS# as of 01/2021.

## Generic Interfaces & Non-Unity Applications ##
* In ROS#, the generic .NET code and Unity-specific code is separated (cf. [Libaries](https://github.com/siemens/ros-sharp/tree/master/Libraries), [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D)) to enable easy re-use of code for non-Unity .NET applications.
* ROS# provides [IProtocol](https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/Protocols/IProtocol.cs) and [ISerializer](https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/Serializers/ISerializer.cs) interfaces to easily add or modify the way of communication and serialization.
* Both libraries and interfaces have been removed in Unity Robotics Hub which simplifies code for standard Unity applications.

## Direct TCP-based Communication vs. RosBridgeSuite ##
* ROS# currently provides (JSON and BSON) WebSocket-connections via RosBridgeSuite only.
* Via [TcpEndpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/) and [TcpConnector](https://github.com/Unity-Technologies/ROS-TCP-Connector) Unity Robotics Hub provides a standalone, tcp-based communication to ROS.
* Unity's direct communication of binary data is faster than the default JSON-based RosBridgeSuite communication, especially for large image data. Further info [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/faq.md) and [here](https://github.com/RobotWebTools/rosbridge_suite/issues/546).
* We are planning to add faster (direct or CBOR-based) [IProtocol](https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/Protocols/IProtocol.cs) implementations and accept pull requests in that direction.

## URDF Import from ROS System ##
* ROS# allows the direct import and export of URDF models and related meshes from a ROS system.
* In Unity Robotics Hub, along with separating [ROS communication](https://github.com/Unity-Technologies/ROS-TCP-Connector) and [URDF importing](https://github.com/Unity-Technologies/URDF-Importer) this feature has been removed.

## Articulation Bodies ##
* Unity Robotics Hub supports [Articulation Bodies](https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody.html).
* ROS# we currently dos not support articulation bodies as they don't allow modeling closed kinematic chains (e.g. delta robots).
* We expect that articulation bodies will be included in PhysX / Unity Editor in a more user-friendly way in future releases, e.g. via a toggles, or physics engines decide by themselves when the use of articulation bodies is appropriate.

---

© Siemens AG, 2021
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
