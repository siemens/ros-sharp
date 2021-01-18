## Unity Robotics has forked the ROS# repo into two new repos:

### [Unity-Technologies/ROS-TCP-Connector](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fgithub.com%2FUnity-Technologies%2FROS-TCP-Connector&data=04|01|martin.bischoff%40siemens.com|7295f464d59b4f18d05d08d8b9ad089c|38ae3bcd95794fd4addab42e1495d55a|1|0|637463500116656222|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|1000&sdata=8fWdBWdYDpuImkuVxnP3vhj94aDirE%2BeKqrRvi%2BudOk%3D&reserved=0)

is a Unity package for sending/receiving messages to/from ROS. It contains extra functionality that, when generating a C# class from a ROS message, functions are also generated that will serialize and deserialize the messages as ROS would internally.

This package, when used with [Unity-Technologies/ROS-TCP-Endpoint](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fgithub.com%2FUnity-Technologies%2FROS-TCP-Endpoint&data=04|01|martin.bischoff%40siemens.com|7295f464d59b4f18d05d08d8b9ad089c|38ae3bcd95794fd4addab42e1495d55a|1|0|637463500116656222|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|1000&sdata=G7%2Fd5jNpjySjU8l7uE7fl27jF5ssawbOc4rH21ofdcY%3D&reserved=0), can increase the speed of message-passing between ROS and Unity, which may be especially important when messages contain images. The tradeoff is that ROS-TCP-Endpoint is not as general as ROS# with rosbridge and has the strict requirement that all messages be serialized by the ROS-TCP-Connector code.

### [Unity-Technologies/URDF-Importer](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fgithub.com%2FUnity-Technologies%2FURDF-Importer&data=04|01|martin.bischoff%40siemens.com|7295f464d59b4f18d05d08d8b9ad089c|38ae3bcd95794fd4addab42e1495d55a|1|0|637463500116666119|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|1000&sdata=R6edPeF1iCF6tARS6X%2Fm9RM6iXl8mZB6M412375NscI%3D&reserved=0)

is a Unity package with added support for instantiating a robot from URDF into a Unity scene with[ Articulation Body](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fblogs.unity3d.com%2F2020%2F05%2F20%2Fuse-articulation-bodies-to-easily-prototype-industrial-designs-with-realistic-motion-and-behavior%2F&data=04|01|martin.bischoff%40siemens.com|7295f464d59b4f18d05d08d8b9ad089c|38ae3bcd95794fd4addab42e1495d55a|1|0|637463500116666119|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|1000&sdata=5deLUo73r%2FvThtFzAnNtZgsRKOZSEbFVL3qsWa3ONaU%3D&reserved=0) components.

Visit the [Unity Robotics Hub](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fgithub.com%2FUnity-Technologies%2FUnity-Robotics-Hub&data=04|01|martin.bischoff%40siemens.com|7295f464d59b4f18d05d08d8b9ad089c|38ae3bcd95794fd4addab42e1495d55a|1|0|637463500116676074|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|1000&sdata=dMB9FbUDJCp0ztdf2AHVDLe5ayjFig0hOnjNH2RiZHY%3D&reserved=0) for more information on how to use these packages.

---

© Unity Technologies, 2021

Author: Cameron Greene ([cameron.greene@unity3d.com](mailto:cameron.greene@unity3d.com))
