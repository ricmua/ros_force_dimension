
<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

The Force Dimension ROS2 node accepts commands and provides feedback data via 
ROS2 [topics][ros_topics].

## Topic namespace and fully-qualified names

Topics are identified by fully-qualified 
[ROS2 names][ros2_name_constraints].[^ros_names]

[^ros_names]: Also see the documentation for [ROS names][ros_names].

All fully-qualified topic names are prefixed with a 
[namespace](doc/markdown/namespace.md). By default, the namespace for the 
Force Dimension node is ``robot``. In this documentation, the node namespace is 
excluded, so that the fully-qualified topic name ``/robot/feedback/position`` 
would here be referred to as ``feedback/position``.

## Feedback topics

A subset of topics provide information about the state of the Force Dimension 
robot. These topics are organized under the ``feedback`` relative namespace. 
The following topics are currently implemented:

* ``feedback/position``: Carries [Point][geometry_msgs_point] messages that 
  describe the current position of the robotic end-effector (as returned by the 
  ``dhdGetPosition`` function of the Force Dimension SDK).
* ``feedback/button``: Carries [Int32][example_interfaces_int32] messages that 
  describe the current status of any buttons on a Force Dimension device. The 
  data element of each such message is a 32-bit integer bitmask (as returned by the ``dhdGetButtonMask`` function of the Force Dimension SDK), where each bit is 
  toggled when the corresponding button is pressed .
* ``feedback/velocity``
* ``feedback/force``
* ``feedback/orientation``
  
Feedback topics can be decimated, in order to reduce the number of sent 
messages. See the [parameters documentation](doc/markdown/parameters.md) for 
information about feedback decimation.






[example_interfaces_int32]: https://docs.ros2.org/latest/api/example_interfaces/msg/Int32.html

[geometry_msgs_point]: http://docs.ros.org/en/latest/api/geometry_msgs/html/msg/Point.html

[ros_topics]: https://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html

[ros2_name_constraints]: http://design.ros2.org/articles/topic_and_service_names.html#ros-2-topic-and-service-name-constraints

[ros_names]: http://wiki.ros.org/Names

