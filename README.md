<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Force Dimension ROS2 package

This package implements a simple ROS2 node for interfacing with Force Dimension 
[haptics][haptics_wikipedia] robots, with the objective of simplifying the 
process of developing applications involving haptic devices. For further 
background information, please see the documentation related to 
[Force Dimension](doc/markdown/force_dimension.md) and 
[ROS](doc/markdown/ros.md).

This package was designed for [ROS2 Galactic][ros2_galactic], and should work equally well with Microsoft Windows or Ubuntu Linux.

[haptics_wikipedia]: https://en.wikipedia.org/wiki/Haptic_technology
[ros2_galactic]: https://docs.ros.org/en/galactic/index.html

## Getting started

Ensure that the package is properly [installed](doc/markdown/installation.md) 
and that the robots are [calibrated](doc/markdown/calibration.md). The package 
can then be used -- in a [configured ROS2 environment][ros2_environment] -- 
like any other [ROS2 package][ros2_package_usage]. For example, a Force 
Dimension server node can be invoked and queried from the 
[command line](doc/markdown/command_line.md).

When designing client nodes that can interact with the Force Dimension server 
node, the [Python example](doc/markdown/python_client_example.md) is a good 
place to start. Also see the documentation related to the 
[topics](doc/markdown/topics.md) and [parameters](doc/markdown/parameters.md) 
associated with the ROS2 node.

## Additional documentation

* [Topics](doc/markdown/topics.md)
* [Parameters](doc/markdown/parameters.md)
* [Calibration](doc/markdown/calibration.md)
* [Launch](doc/markdown/launch.md)
* [Gripper](doc/markdown/gripper.md)
* [Gravity compensation](doc/markdown/gravity_compensation.md)

## License

Copyright 2022-2023 [Neuromechatronics Lab][neuromechatronics], 
Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.



[ros2_package_usage]: https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#use-the-package

[neuromechatronics]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
