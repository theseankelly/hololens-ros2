# ROS2 for HoloLens

`hololens-ros2` is a UWP component library written in the C++/WinRT language
projection for the Windows Runtime. The goal of this library is to provide
interactions with a ROS2 network through a native instance of ROS2 (as opposed
to through `rosbridge`).

The primary motivation is to expose HoloLens data (pose, spatial information,
research mode sensor data, etc) to other participants in the ROS2 network.

This repository consists of the following projects:

* [HoloLensRos2](HoloLensRos2): The UWP Component Library
* [HoloLensRos2Example](HoloLensRos2Example): A simple C# UWP application
demonstrating usage of the `HoloLensRos2` component library.

## Features

* ROS2 Dashing
* `Node` class: An implementation of the [minimal_publisher](https://github.com/ros2/examples/tree/master/rclcpp/minimal_publisher)
demo project

## What's Missing?

* Support for publishing HoloLens pose & spatial awareness information
* Support for publishing raw sensor data (via [Research Mode](https://docs.microsoft.com/en-us/windows/mixed-reality/research-mode))

NOTE: This is not a comprehsensive list of open items. Please see the issue
tracker for full details.

## Getting Started

### Building

This project requires a custom build of ROS2, including various workarounds to
accomodate restrictions of UWP containers (env vars, dynamic loading, etc).

TODO: Provide full build instructions for ROS2.

### Usage

Consuming the `HoloLensRos2` component in your UWP/Unity project is a matter
of adding a reference to the generated `HoloLensRos2.winmd` file and including
the necessary ROS2 dlls to the package.

The UWP application's capabilities must be set to enable `Internet (Client & Server)`
as well as `Private Networks`.

TODO: More explicit, detailed setup instructions are needed.

## Related Work

If you're interested in this project, you may also be interested in the
following projects:

* [ros2_dotnet](https://github.com/ros2-dotnet/ros2_dotnet): aims to bring
C#/.NET language support to ROS2.
  * This project is a great option if your goal
  is to publish/subscribe to artitrary ROS2 topics using standard ROS2 message
  types.
  * `hololens-ros2` has chosen to use `C++/WinRT` and `rclcpp` directly in
  order to take advantage of other existing C++ libraries (such as
  [image_transport](http://wiki.ros.org/image_transport))
  * `hololens-ros2` is designed to compliment and work alongside `ros2_dotnet`.
* [ros-sharp](https://github.com/siemens/ros-sharp): provides a C# binding to
ROS Networks via the (rosbridge)[http://wiki.ros.org/rosbridge_suite]
