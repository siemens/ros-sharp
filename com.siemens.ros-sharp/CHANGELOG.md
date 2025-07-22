Please see [releases](https://github.com/siemens/ros-sharp/releases).

# Changelog

All notable changes to this project will be documented in this file. 
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- Unreleased -->

## [2.2.0] - 22.07.2025

### Added
- **ROS2 Message/Service/Action Generation**: Major refactor and extension of the message auto-generation pipeline to support new ROS2 message, service, and action interface features, including:
    - Support for default values, bounded variable-size arrays and strings in ROS2 `.msg` files. [For example](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html):
        - `string full_name "John Doe"`
        - `int32[] samples [-200, -100, 0, 100, 200]`
        - `int32[<=5] up_to_five_integers_array`
        - `string<=10 up_to_ten_characters_string`
    - New token types and parsing logic for ROS2 message syntax.
    - Improved handling of C# keywords and identifier validation in generated code. Invalid identifiers are added with '`@`' instead of '`_`', eliminating the need for an additional `JsonProperty`.
    - Note: Bounded strings in bounded variable-size arrays are not supported (`string<=10[<=5] up_to_five_strings_up_to_ten_characters_each` for example). These will be generated as standard string arrays. 
- **Editor Usability**: Improved error messages, dialog titles, and progress reporting in Unity Editor message generation windows.

### Changed
- **Refactored Message Parser/Tokenizer**: Message parsing and tokenization logic has been refactored for maintainability, extensibility, and ROS2 compatibility.
    - All API calls below `Parse()` and `Tokenize()` have changed. `Parse()` and `Tokenize()` have the same definition and functionality, and are backwards compatible.
    - `MessageTokenizer.cs` now processes messages line by line using regular expressions instead of character by character.
- **Consistent Constructor Generation**: Parameterized and default constructors in generated message/service/action classes now consistently support new ROS2 features.

### Fixed
- Some of the auto-generated messages now correctly define ROS2-specific arrays.

### Removed
- `CodeDomProvider` based identifier validation is no longer supported. 

## [2.1.1] - 02.06.2025

### Added
-  Multi-framework targeting support, new .NET Standard 2.1 alongside .NET 8.0.

- Updated dependencies, including but not limited to:
    - Microsoft.Bcl.AsyncInterfaces 9.0.5.
    - NUnit 4.3.2.
    - NUnit3TestAdapter 5.0.0.
    - System.Runtime.CompilerServices.Unsafe 6.1.2.
    - System.Text.Encodings.Web 9.0.5.
    - System.Text.Json 9.0.5.
    - System.Threading.Channels 9.0.5.
    - System.IO.Pipelines 9.0.5.
    - websocket-sharp 1.0.3-rc11

### Fixed
- Separate *Editor* and *Runtime* codes (`UnityFibonacciAction*.cs`), which caused build errors for Android and Windows.

- Various wiki enhancements. 

## [2.1.0] - 20.01.2025

### Added
- **ROS2 Action Support**:
    - In **ActionClient.cs** and **ActionServer.cs**, new methods were introduced for setting, sending, canceling, and responding to action goals, accommodating ROS2-specific parameters like feedback, fragment size, and compression, etc. Additionally, **Communicators.cs** now includes new classes—**ActionConsumer** and **ActionProvider**—which facilitate ROS2 action goal management and feedback/result handling (including non-blocking listener callbacks, custom delegates, and more). 
        - Derived example classes **FibonacciActionClient.cs** and **FibonacciActionServer.cs** and their console examples are updated as well. 
    - **ActionServer.cs** now features the ROS2 server state machine model ([ROS2 Design - Actions](https://design.ros2.org/articles/actions.html))  

    - **Namespace and Message Type Integration**:
        - Across key files such as **ActionAutoGen.cs**, **FibonacciActionClient.cs**, and **FibonacciActionServer.cs**, namespaces were updated to `RosSharp.RosBridgeClient.MessageTypes.Action`, aligning with ROS2. Furthermore, message types and identifiers were modified to reflect ROS2 constructs.

    - **Abstract Action Message Types**:
        - In **ActionFeedback.cs**, **ActionResult.cs**, and **ActionGoal.cs**, (`TActionFeedback/Result/Goal`) new properties were added to support ROS2's (action) feedback system and result tracking, enhancing metadata management.

    - **Utility and Infrastructure Enhancements**:
        - **RosSocket.cs** now feature new methods for ROS2 action advertising, unadvertising, managing action goals, cancel requests; feedback, result and status responses.
        - **Communication.cs** now feature ROS2 action message structures defined by the [ROS Bridge Protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md#rosbridge-v20-protocol-specification).  

    - **MessageGeneration Support for ROS2 Actions**:
        - In **ActionAutoGen.cs**, automatic action message generation capabilities have been implemented, including dynamic package inclusion, default and parameterized constructors based on ROS version.

    - **ROS2 Action Support for Unity Fibonacci Examples**


### Changed/Removed
- There were no changes/removals affecting ROS1 functionalities. All new ROS2 enhancements were contained within conditional compilation directives, thereby maintaining the continuity of ROS1 operations.

---


## [2.0.0] - 26.08.2024

### Added

- Unity package supports UPM and requires Unity 2022.3.
- ROS2 support is available for both the Unity Package `(com.siemens.ros-sharp)` and the complete ROS# .NET solution, including RosBridgeClient, MessageGeneration, and Urdf `(Libraries)`.
- New ROS packages: File Server, Unity Simuatlion Scene, and Gazebo Simulation Scene, with ROS2 support.
- Unity Simulation Scene and Gazebo Simulation Scene, included in the Unity Package for ROS2 support.
- RawImageSubscriber script is now part of the Unity package.
- New quick start page.
- Post-build events for Visual Studio streamline development between Unity and .NET.
- Thread safety for `Subscriber : RosBridgeClient.Communication`. Each Subscriber, including .NET and Unity package, can be configured to receive thread-safe. 

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

© Siemens AG, 2017-2025

