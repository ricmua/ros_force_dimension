<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Force Dimension ROS2 package

This package implements a simple ROS2 node for interfacing with 
[Force Dimension](https://www.forcedimension.com) haptics robots. This package 
was designed for [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html), 
and should work equally well on Microsoft Windows or Linux.

## Installation

See the [installation guide](doc/markdown/installation.md).

## Usage

Ensure that the package is properly [installed](doc/markdown/installation.md) 
and that the robots are [calibrated](doc/markdown/calibration.md). The package 
can then be used -- in a [configured ROS2 environment][ros2_environment] -- 
like any other [ROS2 package][ros2_package_usage]. For example:

```
ros2 run force_dimension node
```

To run the node without activating any robots, add the ``disable_hardware`` 
flag:

```
ros2 run force_dimension node --ros-args -p disable_hardware:=True
```

That the node is functioning properly can be confirmed by running the following
command in a second configured ROS2 environment:

```
ros2 topic echo /robot/feedback/position
```

The output will show the ``x``, ``y``, and ``z`` coordinates of the effector 
endpoint (with arbitrary values, if the robot is disabled).

## Additional documentation

* [Topics](doc/markdown/topics.md)
* [Parameters](doc/markdown/parameters.md)
* [Calibration](doc/markdown/calibration.md)
* [Launch](doc/markdown/launch.md)
* [Gripper](doc/markdown/gripper.md)

## License

Copyright 2022 [Neuromechatronics Lab][neuromechatronics], 
Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.



[ros2_package_usage]: https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#use-the-package

[neuromechatronics]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
