<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

In a [configured ROS2 environment][ros2_environment], enter the following 
command to start a Force Dimension (server) node:

```
ros2 run force_dimension node
```

Sometimes, it is necessary to test the node and interface without hardware 
interaction. To run the node without activating any robots, add the 
``disable_hardware`` flag:

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


[ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

