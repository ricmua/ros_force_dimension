
<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->


A [Python client example](../../scripts/run_example) is included to 
illustrate how a client might interact with the Force Dimension node. The 
client is designed to be as simple as possible: it simply prints the current 
3D position of the robot end-effector to the ROS log, each time it receives a 
message on the position topic (default: ``feedback/position``).

In order to make it easier to run the example, a 
[launch file](../../launch/example.launch.py) has been provided. Once the ROS2 
workspace has been built, the example can be started with the following 
command:

```ros2 launch force_dimension example.launch.py```

NOTE: At present, it is necessary that a Force Dimension robot is available to 
the Force Dimension node at launch. Otherwise, the node will fail.

