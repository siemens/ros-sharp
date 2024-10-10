Please see [releases](https://github.com/siemens/ros-sharp/releases).

# Changelog

All notable changes to this project will be documented in this file. 
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- ## [Unreleased] - xx.xx.2024 -->

## [2.0.0] - 26.08.2024

### Added

- Unity package supports UPM and requires Unity 2022.3.
- ROS2 support is available for both the Unity Package `(com.siemens.ros-sharp)` and the complete ROS# .NET solution, including RosBridgeClient, MessageGeneration, and Urdf `(Libraries)`.
- New ROS packages: File Server, Unity Simuatlion Scene, and Gazebo Simulation Scene, with ROS2 support.
- Unity Simulation Scene and Gazebo Simulation Scene, included in the Unity Package for ROS2 support.
- RawImageSubscriber script is now part of the Unity package.
- New quick start page.
- Post-build events for Visual Studio streamline development between Unity and .NET.

### Fixed

- UrdfTransfer files use serializer-specific methods instead of Newtonsoft JSON.

### Changed

- RosBridgeClient, MessageGeneration, and Urdf support .NET 8.0.
- RosBridgeClient, MessageGeneration, and Urdf source code is included in the Unity Package; and no longer dynamically linked to the Unity package.
- Switched from websocket-sharp to websocket-sharp.netstandard.
- The JoyAxisReader script in the Unity package inherits from IAxisReader interface for increased applicability.
- The ImageSubscriber script in the Unity package is renamed to CompressedImageSubscriber.
- URDF export and import windows in the Unity package utilize the existing RosSocket component in the scene.  

### Removed

- Newtonsoft BSON is no longer supported.  

---

© Siemens AG, 2017-2024

