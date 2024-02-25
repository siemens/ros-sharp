Please see https://github.com/siemens/ros-sharp/releases

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0] - 23.02.2024

### Added

- Unity package inherits UPM and requires Unity 22.3
- TODO: Both Unity Package and RosBridgeClient supports ROS2.

### Fixed

- UrdfTransfer files uses the 'selected' serializer specific methods instead of Newtonsoft JSON.

### Changed

- RosBridgeClient now supports .NET 8.0
- RosBridgeClient is no longer dynamically linked to the Unity package. 
- Moved from websocketsharp to websocketsharp.netstandard

### Removed

- Newtonsoft BSON is no longer supported.  


[unreleased]: https://github.com/olivierlacan/keep-a-changelog/compare/v1.1.1...HEAD
