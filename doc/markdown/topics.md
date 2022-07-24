<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

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
[namespace](namespace.md). By default, the namespace for the 
Force Dimension node is ``robot``. In this documentation, the node namespace is 
excluded, so that the fully-qualified topic name ``/robot/feedback/position`` 
would here be referred to as ``feedback/position``.

## Feedback topics

A subset of topics provide information about the state of the Force Dimension 
robot. These topics are organized under the ``feedback`` relative namespace. 
The following topics are currently implemented:

* ``feedback/position``: Carries [Point][geometry_msgs_point] messages that 
  describe the current position of the robotic end-effector (as returned by the 
  [dhdGetPosition] function of the Force Dimension SDK).
* ``feedback/button``: Carries [Int32][example_interfaces_int32] messages that 
  describe the current status of any buttons on a Force Dimension device. The 
  data element of each such message is a 32-bit integer bitmask (as returned by 
  the [dhdGetButtonMask] function of the Force Dimension SDK), where each bit 
  is toggled when the corresponding button is pressed .
* ``feedback/gripper_gap``: Carries [Float64][example_interfaces.float64] 
  messages that describe the current distance of the gripper opening in meters. 
  See the [dhdGetGripperGap] function of the Force Dimension SDK documentation 
  for further details.
* ``feedback/gripper_angle``: Carries [Float64][example_interfaces.float64] 
  messages that describe the current distance of the gripper opening angle in 
  radians. See the [dhdGetGripperAngleRad] function of the Force Dimension SDK 
  documentation for further details.
* ``feedback/velocity``
* ``feedback/force``
* ``feedback/orientation``

State feedback is published at regular intervals. See the 
[sampling interval](doc/markdown/parameters.md#sampling-interval) parameters 
documentation for more information about the publication interval.

Feedback topics can be decimated, in order to reduce the number of sent 
messages. See the [parameters documentation](parameters.md) for 
information about feedback decimation.

## Command topics

* ``command/force``: Carries [Vector3][geometry_msgs_vector3] messages that 
  indicate a force vector to apply to the robotic end-effector (as 
  accomplished via the [dhdSetForce] function of the Force Dimension SDK).
* ``command/gripper_force``: Carries [Float64][example_interfaces.float64] 
  messages that indicate a force vector to apply to the robotic gripper (as 
  accomplished via the [dhdSetForceAndGripperForce] function of the Force 
  Dimension SDK). Note that the Force Dimension SDK does not decouple gripper 
  force commands from endpoint force commands. Messages sent to this topic 
  automatically trigger an endpoint force command. See the
  [gripper documentation](doc/markdown/gripper.md) for further details.

[dhdGetGripperAngleRad]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#aacb9cbecf42f01330bd9a8fc512011d9

[dhdGetPosition]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#ac6910076186b2709dec3c2bfa38628c2

[dhdGetButtonMask]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a5fbdfdb991ebe0faa92f1bcaffde5a75

[dhdGetGripperGap]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#ac8e059defb0d2e3255e4f74cf941e4eb

[example_interfaces.float64]:https://docs.ros2.org/latest/api/example_interfaces/msg/Float64.html

[example_interfaces_int32]: https://docs.ros2.org/latest/api/example_interfaces/msg/Int32.html

[geometry_msgs_point]: http://docs.ros.org/en/latest/api/geometry_msgs/html/msg/Point.html

[geometry_msgs_vector3]: http://docs.ros.org/en/latest/api/geometry_msgs/html/msg/Vector3.html

[ros_topics]: https://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html

[ros2_name_constraints]: http://design.ros2.org/articles/topic_and_service_names.html#ros-2-topic-and-service-name-constraints

[ros_names]: http://wiki.ros.org/Names

[dhdSetForce]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#abb6f549b805a44a23a78aa6c29085c28

[dhdSetForceAndGripperForce]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#aa2beb27a94c693149603619d44fc2725

