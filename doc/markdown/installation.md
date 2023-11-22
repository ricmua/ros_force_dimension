<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Installation

Installation of this ROS2 package has been tested with Ubuntu 20.04, 
Ubuntu 22.04, and Windows 10.

## Force Dimension SDK

In order to build the Force Dimension ROS2 node, the Force Dimension SDK must 
be available to the [ROS2 build system]. The SDK provides all drivers and 
libraries that are essential for interfacing with a Force Dimension robot. See 
the [supporting documentation](/doc/markdown/installation-sdk.md) for more 
information.

## ROS2 package

The Force Dimension ROS2 package is built in the 
[same way][build_a_ros2_package] as an other ROS2 package, using `colcon`.[^4] 

[^4]: Reminder: The `ForceDimensionSDK_DIR` environmental variable must be 
      set during the build process. See the 
      [instructions](#force-dimension-sdk) for installing the Force Dimension 
      SDK.





<!------------------------------------------------------------------------------
  REFERENCES
------------------------------------------------------------------------------->

[build_a_ros2_package]: https://docs.ros.org/en/iron/Tutorials/Creating-Your-First-ROS2-Package.html#build-a-package

[ROS2 build system]: https://docs.ros.org/en/iron/Concepts/About-Build-System.html


