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
and [calibrated](doc/markdown/calibration.md). The package can then be used 
like any other [ROS2 package][ros2_package_usage]. For example:

```
ros2 run force_dimension node
```

The [Python example](#python-example) is a good place to start.

### Python example

A Python client node example is provided. See the 
[documentation](doc/markdown/python_client_example.md), or go directly to the 
[code](scripts/run_example).

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

