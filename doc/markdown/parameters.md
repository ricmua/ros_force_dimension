<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Parameters

A number of [ROS2 parameters][ros2_parameters] can be used to configure the 
operation and behavior of the Force Dimension node. These parameters can be 
modified programmatically, via configuration files, on the command line 
[at startup][command_line_parameters], or via ``ros2 param`` calls.

### Sampling interval

The sampling frequency defines how frequently the node updates. The 
ROS2 parameter that determines the sampling frequency is ``sample_interval_s``. 
This parameter takes floating point values, in units of seconds.

In order to ensure smooth haptics and stable application of forces, 
the Force Dimension robots should operate at a frequency in the kilohertz 
range.[^1] The recommended update rate is `2000 Hz` (corresponding to a 
parameter value of ``sample_interval_s: 0.0005``).

[^1]: Reference requested.

### Feedback sample decimation

Reporting feedback and storing data at a high sampling frequency can result 
in busy transmission channels and large / cumbersome data files. For this 
reason, the Force Dimension node accommodates decimation of message publication 
on the feedback topics. The parameters that determine the decimation factor are contained within the ``feedback_sample_decimation`` parameter 
namespace.[^parameter_namespace]

Each parameter consists of an integer that represents a decimation factor 
for each feedback variable. This integer determines the number of samples or 
update cycles between each published message. That is, a message is published 
each time the sample counter can be evenly-divided by this factor. If the 
parameter value is ``1``, then no decimation occurs. If the parameter value is 
less than ``1``, then no messages are published at all (i.e., the feedback 
channel / topic is disabled).

As an example, consider the ROS topic that publishes the position of the 
robotic end-effector (e.g., ``/robot/feedback/position``). If the sampling 
/ update rate of the node is ``2000 Hz`` (i.e., ``sample_interval_s: 0.0005``), 
then a decimation factor of ``50`` (i.e., 
``feedback_sample_decimation.position: 50``) would result in position 
feedback published at ``40 Hz``.

[^parameter_namespace]: ROS2 parameter namespaces do not seem to be 
                        well-documented. Parameter namespace prefixes are 
                        separated from parameter names by a period, and all 
                        parameters in a namespace can be 
                        [retrieved][get_parameters_by_prefix] at once. In 
                        YAML configuration files, parameter namespaces are 
                        specified via indentation.

[get_parameters_by_prefix]: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.get_parameters_by_prefix

### Hardware switch

The ``disable_hardware`` ROS2 parameter is a boolean value that determines 
whether or not Force Dimension SDK calls are passed through. If the parameter 
value is ``true``, then no SDK calls are made, and interaction with any robots 
is therefore disabled. The node continues to operate and publish data, even in 
the absence of calls to the SDK. This can be useful for testing and / or 
debugging, in cases where a robot is not needed or not available.

### Button emulation

It is possible to treat the gripper of Force Dimension robots that lack buttons 
as though it were a button. See the [dhdEmulateButton] function of the Force 
Dimension haptics SDK for further details. The ``gripper.emulate_button`` ROS2 
parameter toggles this capability. Button emulation is enabled or disabled 
only during the activation process, so that changing the parameter while the 
Force Dimension node is active will have no effect.

## Sample configuration file

ROS2 parameters can be [dumped and loaded][ros2_yaml_parameters] to/from a 
[YAML] configuration file. Here is a sample configuration file for the 
Force Dimension node:

```
/robot/force_dimension:
  ros__parameters:
    sample_interval_s: 0.0005
    feedback_sample_decimation:
      position: 50
      button: 50
      gripper_gap: 50
      gripper_angle: 50
      velocity: 200
    disable_hardware: false
    gripper:
      emulate_button: false
    gravity_compensation: true
    effector_mass_kg: 0.279
```


[YAML]: https://en.wikipedia.org/wiki/YAML

[ros2_yaml_parameters]: https://docs.ros.org/en/humble/Tutorials/Parameters/Understanding-ROS2-Parameters.html#ros2-param-dump

[ros2_parameters]: https://docs.ros.org/en/humble/Tutorials/Parameters/Understanding-ROS2-Parameters.html
 
[command_line_parameters]: https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html#parameters

[dhdEmulateButton]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#aa23bc77b009e020c05ef725eb533b60c


