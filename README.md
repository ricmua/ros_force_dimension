<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Force Dimension ROS2 package

This package implements a simple ROS2 node for interfacing with Force Dimension 
[haptics] robots, with the objective of simplifying the process of developing 
applications involving haptic devices. For further background information, 
please see the internal documentation related to
[Force Dimension](doc/markdown/force_dimension.md) and 
[ROS](doc/markdown/ros.md).

This package has been tested with [ROS2 Galactic], [ROS2 Humble], and 
[ROS2 Iron]. It has been tested with Ubuntu Linux and Microsoft Windows, as 
well as Windows Subsystem for Linux (WSL2).

## Getting started

Ensure that the package is properly [installed](doc/markdown/installation.md) 
and that the robots are [calibrated](doc/markdown/calibration.md). The package 
can then be used -- in a [configured ROS2 environment] -- like any other 
[ROS2 package]. For example, a Force Dimension server node can be invoked and 
queried from the [command line](doc/markdown/command_line.md).

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

Copyright 2022-2023 [Neuromechatronics Lab], Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.



<!------------------------------------------------------------------------------
  REFERENCES
------------------------------------------------------------------------------->

[haptics]: https://en.wikipedia.org/wiki/Haptic_technology

[ROS2 Galactic]: https://docs.ros.org/en/galactic/index.html


[ROS2 Iron]: https://docs.ros.org/en/iron/index.html

[ROS2 Humble]: https://docs.ros.org/en/humble/index.html


[ROS2 package]: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

Neuromechatronics Lab]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[configured ROS2 environment]: https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

